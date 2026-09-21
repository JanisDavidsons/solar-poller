// solar_poller.cpp
//
// Polls an Eastron SDM630-Modbus V2 energy meter via Modbus RTU,
// prints per-phase voltage/current/power and total net power to stdout,
// and publishes each value to MQTT under the home/solar/* topic tree.
//
// Build:  g++ -O2 -Wall -o solar_poller solar_poller.cpp -lmodbus -lmosquitto
// Run:    ./solar_poller
// Watch:  mosquitto_sub -h localhost -t 'home/solar/#' -v
//
// Addressing note (trap for the unwary):
//   mbpoll command line uses 1-indexed register numbers.
//   libmodbus C API uses 0-indexed register numbers.
//   So mbpoll "-r 1" == modbus_read_input_registers(ctx, 0, ...).
//   All register constants below are 0-indexed (libmodbus style).

#include <modbus/modbus.h>
#include <mosquitto.h>

#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <gpiod.h>
#include <unistd.h>

// ---------- Config ----------
constexpr const char *SERIAL_PORT = "/dev/sdm630";
constexpr int MODBUS_BAUD = 9600;
constexpr char MODBUS_PARITY = 'N';
constexpr int MODBUS_DATA_BITS = 8;
constexpr int MODBUS_STOP_BITS = 1;
constexpr int METER_SLAVE_ADDR = 1;
constexpr int POLL_INTERVAL_SEC = 5;

constexpr const char *MQTT_HOST = "localhost";
constexpr int MQTT_PORT = 1883;
constexpr const char *MQTT_CLIENT_ID = "solar_poller";
constexpr const char *MQTT_TOPIC_PREFIX = "home/solar";

// ---------- Heater control config ----------
constexpr const char *GPIO_CHIP_PATH = "/dev/gpiochip0";

// Four single-phase resistive elements, all four switched by one 4-channel
// ACTIVE-LOW relay module (JQC3F-03VDC, 3 V coils) fed from a separate 3.3 V
// PSU that shares ground with the Pi. IN pins go straight to these GPIO
// lines, with no transistor stage, so physical low = relay energised = heat.
//
// The inversion is handled once, in init(), by asking the kernel for
// active_low on the line. Everything above that layer keeps writing
// 1 = heat / 0 = off, and no decision logic knows about the polarity.
//
// Controller names are PHASE names, not element positions: "L2" means the
// controller deciding on p_l2, whichever element happens to hang off it.
//
// Full line -> phase -> element mapping, measured live 2026-09-21 with relay
// LEDs against per-phase power jumps. This replaces the assumed mapping
// carried since 2026-08-31, under which L1b and L3 had their elements the
// wrong way round (top vs middle) and three of the four line numbers were
// wrong. GPIO line numbers are gpiochip0 offsets.
//
//   ctrl  line  pin        phase  element               R        load
//   L1    9     PA9  (33)  L1     domestic hot water    44.7 ohm 1245 W
//   L1b   7     PA7  (29)  L1     buffer MIDDLE         45.0 ohm 1240 W
//   L2    10    PA10 (35)  L2     buffer BOTTOM         35.7 ohm 1560 W
//   L3    8     PA8  (31)  L3     buffer TOP            44.7 ohm 1245 W
//
// L1 and L1b still share phase L1, so the gating is unchanged: L1b only runs
// when the hot water tank is at its cap and the L1 primary is therefore off
// (see gating in main loop). Sensor and cap bindings follow the element's
// role, not its line: DHW takes temp_hot_water and the 70 C cap, all three
// buffer elements take temp_upper_buf and the 85 C cap.
constexpr unsigned int HEATER_L1_GPIO_LINE = 9;
constexpr unsigned int HEATER_L1B_GPIO_LINE = 7;
constexpr unsigned int HEATER_L2_GPIO_LINE = 10;
constexpr unsigned int HEATER_L3_GPIO_LINE = 8;
// ---------- Per-heater element loads ----------
// Each element's own draw sets both of its switching pivots, so these are
// per-heater and meant to be edited in place: change the number and the
// comment on the same line, nothing else needs touching.
//
// Source: cold-resistance measurement of all four elements 2026-09-21,
// P = U^2/R at a measured ~236 V. This supersedes the 2026-08-31
// switch-transition estimates, which had L1 at 1236 W and L2 at 1566 W. The
// two methods agree: 35.7 ohm at 236 V gives 1560 W against the 1566 W
// observed, so the earlier 27% spread was real after all and not measurement
// error. Three elements are 1240-1245 W and one is a 1.5 kW element that was
// sold as a 1.2 kW unit.
//
// Which element each controller drives is measured, not assumed, as of the
// live line-to-phase-to-element test on 2026-09-21; see the mapping table
// above. Keep each constant paired with the element named on its line: if
// the panel is rewired, the table and these move together.
constexpr float HEATER_LOAD_L1_W = 1245.0f;  // DHW, 44.7 ohm @ 236 V, 2026-09-21
constexpr float HEATER_LOAD_L1B_W = 1240.0f; // buffer MIDDLE, 45.0 ohm @ 236 V, 2026-09-21
constexpr float HEATER_LOAD_L2_W = 1560.0f;  // buffer BOTTOM, 35.7 ohm @ 236 V, 2026-09-21; mis-sold 1.5 kW element
constexpr float HEATER_LOAD_L3_W = 1245.0f;  // buffer TOP, 44.7 ohm @ 236 V, 2026-09-21

// Per-phase decision: each heater is judged on the export of its own phase.
// Utility meters per-phase, so a phase has to export enough on its own to
// absorb its heater. Hysteresis timers prevent rapid cycling.
// Caps bind to the element's role, not its GPIO line: the DHW element takes
// the hot water cap, all three buffer elements take the accumulator cap.
constexpr float HOT_WATER_MAX_TEMP = 70.0f;             // °C — L1 (DHW tank) cap
constexpr float BUFFER_MAX_TEMP = 85.0f;                // °C — L1b/L2/L3 (accumulator) cap
// ---------- Break-even economics (VAT-inclusive, cents/kWh) ----------
// Running an element is worth it only when the solar+grid mix costs no more
// than the pellets it displaces. Every kWh diverted to a heater is a kWh we
// stop being paid EXPORT_RATE_C for, so the export rate is the opportunity
// cost of the solar share, not zero.
//
// PELLET_COST_C is the single tuning knob. 6.0 = own pellet stock at
// EUR 255/t, 4.75 kWh/kg net, ~90% burner efficiency. Switch to 8.9f
// (market EUR 380/t) if by spring the EKII heat-pump grant is still
// unapproved and stock won't cover winter 2027/28.
constexpr float PELLET_COST_C = 6.0f;
constexpr float GRID_PEAK_C = 18.09f;
constexpr float GRID_CHEAP_C = 11.60f;
constexpr float EXPORT_RATE_C = 1.03f;

// The solar-fraction formulas below divide by (grid - export) and are only
// meaningful while grid power costs more than the pellets it replaces.
static_assert(GRID_CHEAP_C > PELLET_COST_C,
              "Cheap-window grid price must exceed pellet cost, else grid-only "
              "heating is cheaper and this whole threshold scheme is the wrong "
              "control law.");
static_assert(PELLET_COST_C > EXPORT_RATE_C,
              "Pellets must cost more than the export credit, else exporting "
              "beats self-consumption and no heater should ever run.");
static_assert(GRID_PEAK_C > GRID_CHEAP_C,
              "Peak tariff must exceed the cheap tariff.");

// Both pivots are measured phase power (negative = export), read from
// opposite sides of the switch: turn-on against a phase with the element OFF,
// turn-off against a phase with it ON and drawing its own load_w.
//
// The band that matters is TURN_OFF - draw - TURN_ON, and it has to be
// widened deliberately. Deriving both pivots from one "acceptable import"
// constant (as this once did) only slides the band: the constant appears in
// both terms and cancels out. Here the break-even import is that shared term,
// so the width is exactly TURN_ON_HEADROOM_W + TURN_OFF_HYSTERESIS_W, 300 W,
// the same in both tariff windows AND for every element regardless of its
// load. That last property is why the thresholds are per-heater: with one
// global 1200 W figure, an element drawing 1566 W lands 66 W past its own
// shed pivot and cycles on every marginal switch-on.
//
// TURN_ON_HEADROOM_W is export the phase must show beyond the break-even
// entry point before we switch, so the element lands clear of the shed pivot
// instead of right on it, where any PV flicker would cycle it. 150 W and not
// more: at 300 W the cheap-window entry for a 1.2 kW element moves to -936 W,
// which autumn per-phase export rarely reaches, and the change would buy
// profitability at the price of never running.
constexpr float TURN_ON_HEADROOM_W = 150.0f;
// Shed once a running element imports more than break-even allows, held for
// TURN_OFF_DELAY_SEC so a passing cloud doesn't drop it.
constexpr float TURN_OFF_HYSTERESIS_W = 150.0f;

// Share of an element's load that solar must cover for the solar+grid mix to
// beat pellets; the remainder may be bought from the grid. Independent of
// element size, so it is the one part of this that stays a plain number.
constexpr float solar_frac(float grid_c)
{
    return (grid_c - PELLET_COST_C) / (grid_c - EXPORT_RATE_C);
}
constexpr float SOLAR_FRAC_PEAK = solar_frac(GRID_PEAK_C);   // ~0.709
constexpr float SOLAR_FRAC_CHEAP = solar_frac(GRID_CHEAP_C); // ~0.530

// Grid import a running element may draw and still break even. Always well
// under its full draw, so no element ever runs on grid alone.
constexpr float allowed_import_w(float load_w, float grid_c)
{
    return load_w * (1.0f - solar_frac(grid_c));
}
constexpr float turn_on_threshold_w(float load_w, float grid_c)
{
    return -(load_w - allowed_import_w(load_w, grid_c) + TURN_ON_HEADROOM_W);
}
constexpr float turn_off_threshold_w(float load_w, float grid_c)
{
    return allowed_import_w(load_w, grid_c) + TURN_OFF_HYSTERESIS_W;
}

constexpr int TURN_ON_DELAY_SEC = 30;
constexpr int TURN_OFF_DELAY_SEC = 60;

// If no boiler temperature update arrives within this window, treat the
// last value as untrusted and force the affected heater(s) off. boiler_poller
// publishes every 60s, so 300s = 4 missed cycles of headroom.
constexpr int TEMP_STALE_SEC = 300;

// Consecutive failed meter reads after which every heater is forced off and
// held off. Without the meter there is no per-phase export figure, so there
// is nothing authorising a heater to run: it would be heating on an unknown
// import. Long enough to ride out a transient CRC error, short enough that
// an unmetered element never runs meaningfully.
//
// Note this is counted in failures, not seconds. The retry backoff is
// 5/10/20/30/30 s, so the 6th failure lands roughly 95-105 s after the
// first, not at 6 x POLL_INTERVAL_SEC.
constexpr int METER_FAILURE_LIMIT = 6;

// ---------- SDM630 input-register map (0-indexed) ----------
// Block 1: registers 0..17 (18 registers = 9 floats):
//   V_L1, V_L2, V_L3, I_L1, I_L2, I_L3, P_L1, P_L2, P_L3
// Block 2: registers 52..53 (2 registers = 1 float):
//   Total System Power (net import/export, watts)
constexpr int BLOCK1_START = 0;
constexpr int BLOCK1_COUNT = 18;
constexpr int TOTAL_P_START = 52;
constexpr int TOTAL_P_COUNT = 2;

// ---------- Graceful shutdown ----------
static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running = false; }

// Latest tank temperatures received via boiler_poller MQTT publishes.
// Written by the mosquitto network thread, read by the main loop.
// `seen_at` is 0 until the first message arrives — anything else is the
// wall-clock time of the most recent valid reading.
static std::atomic<float> g_hot_water_temp{0.0f};
static std::atomic<time_t> g_hot_water_temp_seen_at{0};
static std::atomic<float> g_buffer_upper_temp{0.0f};
static std::atomic<time_t> g_buffer_upper_temp_seen_at{0};

// ---------- Helpers ----------

// Combine two consecutive 16-bit registers into one IEEE 754 float.
// SDM630 sends the high word first ("big endian" word order).
// We avoid union/pointer-cast type punning (UB in C++) and use memcpy.
static float regs_to_float(uint16_t high, uint16_t low)
{
    uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
    float result;
    std::memcpy(&result, &combined, sizeof(float));
    return result;
}

static void publish_float(mosquitto *mosq, const char *sub_topic, float value)
{
    char topic[128];
    char payload[64];
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, sub_topic);
    int n = snprintf(payload, sizeof(payload), "%.2f", value);
    mosquitto_publish(mosq, nullptr, topic, n, payload, /*qos=*/0, /*retain=*/false);
}

static void print_timestamp()
{
    time_t now = time(nullptr);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    printf("[%s] ", buf);
}

// Ignitis cheap-tariff windows: [02:00,06:00) and [12:00,16:00) local time.
// Inclusive start, exclusive end, so 06:00 and 16:00 are peak hours.
// Europe/Riga comes from the system timezone. localtime_r is thread-safe,
// unlike the localtime() used in print_timestamp above.
static bool is_cheap_window(time_t now)
{
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    const int h = tm_now.tm_hour;
    return (h >= 2 && h < 6) || (h >= 12 && h < 16);
}

// ---------- Heater controller ----------
//
// One instance per heater. Owns a libgpiod v2 line request on a single
// gpiochip0 offset and runs an on/off state machine driven by per-phase
// export power and a tank temperature cap.
//
// Releasing the line is NOT a safe default on this SoC. gpiochip0 retains
// the last driven value after the request goes away, so the safe state has
// to be driven explicitly before releasing; see shutdown(). Only SIGTERM
// reaches that path, which is why kill -9 is banned for this process.
struct HeaterController
{
    const char *name;       // "L1" / "L2" / "L3" — used in log lines
    unsigned int gpio_line; // gpiochip0 offset for this heater's relay input
    float temp_cap;         // °C — refuse to heat at or above this
    float load_w;           // W — this element's own draw, sets both pivots

    gpiod_chip *chip = nullptr;
    gpiod_line_request *request = nullptr;
    bool on = false;
    time_t below_on_since = 0;
    time_t above_off_since = 0;
    bool temp_first_seen_logged = false;
    bool temp_stale = true; // assume stale until first reading arrives

    bool init()
    {
        chip = gpiod_chip_open(GPIO_CHIP_PATH);
        if (!chip)
        {
            fprintf(stderr, "[%s] gpiod_chip_open(%s) failed\n", name, GPIO_CHIP_PATH);
            return false;
        }

        gpiod_line_settings *settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        // The relay module is active-low: pulling the IN pin to ground
        // energises the coil. Declare that here and the kernel inverts every
        // read and write on the line, so the rest of this file can keep
        // treating ACTIVE as "heat" and INACTIVE as "off".
        //
        // This must be set before the request, not after: the initial output
        // value below is applied through the same inversion, so the line
        // comes up physically high, which is the module's idle state. Without
        // the flag the request itself would switch the heater on.
        gpiod_line_settings_set_active_low(settings, true);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config *line_cfg = gpiod_line_config_new();
        unsigned int offsets[1] = {gpio_line};
        gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);

        gpiod_request_config *req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "solar_poller");

        request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);

        if (!request)
        {
            fprintf(stderr, "[%s] gpiod_chip_request_lines(line %u) failed: %s\n",
                    name, gpio_line, strerror(errno));
            gpiod_chip_close(chip);
            chip = nullptr;
            return false;
        }
        return true;
    }

    void set(bool turn_on)
    {
        if (!request)
            return;
        if (turn_on == on)
            return;
        gpiod_line_request_set_value(request, gpio_line,
                                     turn_on ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
        on = turn_on;
        print_timestamp();
        printf("Heater %s %s\n", name, turn_on ? "ON" : "OFF");
    }

    // Temperature side of the decision, shared by the normal path and the
    // meter-failure hold. Runs the staleness bookkeeping and its transition
    // logging, then enforces the cap.
    //
    // Returns true if temperature permits this heater to run. On a false
    // return the heater has already been driven off and its hysteresis
    // timers cleared, so the caller can simply stop.
    //
    // tank_temp: latest reading (°C) for the tank this heater sits in.
    // tank_temp_seen_at: wall-clock time of that message, 0 = never received.
    // Older than TEMP_STALE_SEC and the heater is forced off: running on a
    // stale reading risks overshooting the cap.
    bool temperature_permits_running(float tank_temp, time_t tank_temp_seen_at,
                                     time_t now)
    {
        bool have_temp = (tank_temp_seen_at != 0);
        bool is_stale = !have_temp ||
                        (now - tank_temp_seen_at >= TEMP_STALE_SEC);

        if (have_temp && !temp_first_seen_logged)
        {
            print_timestamp();
            printf("[%s] tank temp: first reading %.1f°C\n", name, tank_temp);
            temp_first_seen_logged = true;
            temp_stale = false;
        }
        else if (is_stale && !temp_stale)
        {
            print_timestamp();
            printf("[%s] tank temp stale (last update %lds ago) — heater forced OFF\n",
                   name, static_cast<long>(now - tank_temp_seen_at));
            temp_stale = true;
        }
        else if (!is_stale && temp_stale && temp_first_seen_logged)
        {
            print_timestamp();
            printf("[%s] tank temp fresh again: %.1f°C\n", name, tank_temp);
            temp_stale = false;
        }

        if (is_stale)
        {
            if (on)
                set(false);
            below_on_since = above_off_since = 0;
            return false;
        }

        if (tank_temp >= temp_cap)
        {
            if (on)
            {
                print_timestamp();
                printf("Heater %s OFF — tank at %.1f°C (limit %.0f°C)\n",
                       name, tank_temp, temp_cap);
                set(false);
            }
            below_on_since = above_off_since = 0;
            return false;
        }
        return true;
    }

    // p_decision_w: per-phase power (negative = export) for this heater's phase.
    void update(float p_decision_w, float tank_temp,
                time_t tank_temp_seen_at, time_t now)
    {
        if (!temperature_permits_running(tank_temp, tank_temp_seen_at, now))
            return;

        // Tariff window and this element's own load decide both pivots for
        // the cycle. A window change can shed a running element (both pivots
        // tighten at 06:00 and 16:00), which is intended: past that hour the
        // same import costs more than the pellets it displaces.
        const float grid_c = is_cheap_window(now) ? GRID_CHEAP_C : GRID_PEAK_C;
        const float on_threshold = turn_on_threshold_w(load_w, grid_c);
        const float off_threshold = turn_off_threshold_w(load_w, grid_c);

        if (!on)
        {
            if (!request)
                return;
            if (p_decision_w < on_threshold)
            {
                if (below_on_since == 0)
                    below_on_since = now;
                if (now - below_on_since >= TURN_ON_DELAY_SEC)
                {
                    set(true);
                    below_on_since = 0;
                    above_off_since = 0;
                }
            }
            else
            {
                below_on_since = 0; // not sustained — reset
            }
        }
        else
        {
            // Judge on measured power, same frame as the turn-on test above:
            // the element's draw is already baked into p_decision_w, so an
            // import reading past off_threshold means solar is no longer
            // covering enough of it to beat pellets. Don't reconstruct "power
            // if the heater were off" here. That asks whether the phase has
            // *any* surplus rather than whether the element is being paid for,
            // and the heater then stays on while importing ~1 kW.
            //
            // Flap resistance comes from TURN_ON_HEADROOM_W, not from this
            // test: a phase exporting only just enough to feed the element
            // lands right on the break-even import, a hair under this pivot,
            // and any PV flicker then cycles it. The headroom is what keeps
            // the landing point (~+200 W) a 300 W margin away.
            if (p_decision_w > off_threshold)
            {
                if (above_off_since == 0)
                    above_off_since = now;
                if (now - above_off_since >= TURN_OFF_DELAY_SEC)
                {
                    set(false);
                    above_off_since = 0;
                    below_on_since = 0;
                }
            }
            else
            {
                above_off_since = 0;
            }
        }
    }

    // The meter has stopped answering, so the per-phase export figure that
    // normally authorises heating does not exist. Nothing may run: a heater
    // left on here is heating against an unknown import.
    //
    // The temperature bookkeeping still runs, so staleness tracking and its
    // logging stay current through the outage and the cap is still enforced
    // if fresh readings are arriving. Both branches end with the heater off;
    // the point is that the outage does not silently suspend the safety
    // machinery. Timers are cleared too, so when the meter returns the
    // heater re-qualifies through the normal TURN_ON_DELAY_SEC path rather
    // than resuming its previous state.
    void meter_failure_hold(float tank_temp, time_t tank_temp_seen_at, time_t now)
    {
        temperature_permits_running(tank_temp, tank_temp_seen_at, now);
        force_off();
    }

    // One startup line per heater, so the journal records exactly which
    // pivots this binary is running with for each element.
    void log_thresholds() const
    {
        printf("  %-3s load %4.0f W | cheap: on %6.0f / off %5.0f | "
               "peak: on %6.0f / off %5.0f | band %.0f W\n",
               name, load_w,
               turn_on_threshold_w(load_w, GRID_CHEAP_C),
               turn_off_threshold_w(load_w, GRID_CHEAP_C),
               turn_on_threshold_w(load_w, GRID_PEAK_C),
               turn_off_threshold_w(load_w, GRID_PEAK_C),
               turn_off_threshold_w(load_w, GRID_CHEAP_C) - load_w -
                   turn_on_threshold_w(load_w, GRID_CHEAP_C));
    }

    // Drive the line to logical OFF, THEN release it. The order is not
    // cosmetic. This SoC's gpiochip0 retains the last driven value after the
    // request is released (the line stays an output, it does not float or
    // reset), so whatever is on the pin when we let go is what the relay sees
    // until something else claims the line.
    //
    // Observed 2026-09-21: after `systemctl stop`, the kernel held the heater
    // lines as output-LOW. Under the old Ford relays that was OFF and nobody
    // noticed. Under the active-low module it is ALL FOUR HEATERS ON with no
    // process running and nothing watching the tank temperatures, and it took
    // a manual gpioget per line to clear. With active_low declared on the
    // request, the INACTIVE write below leaves the pin physically HIGH, which
    // is the module's idle state, so releasing is now safe.
    //
    // This does not cover SIGKILL: the kernel still retains whatever the pin
    // held at the moment of death, so a heater that was ON stays ON. SIGTERM
    // is the only clean path, which is the other reason not to kill -9 this
    // process.
    void shutdown()
    {
        if (request)
        {
            gpiod_line_request_set_value(request, gpio_line, GPIOD_LINE_VALUE_INACTIVE);
            on = false;
            gpiod_line_request_release(request);
            request = nullptr;
        }
        if (chip)
        {
            gpiod_chip_close(chip);
            chip = nullptr;
        }
    }

    // External hard-off: drop the line and reset hysteresis timers, used when a
    // higher-level rule disqualifies this heater this cycle (e.g. L1b when the
    // hot water tank isn't yet at cap).
    void force_off()
    {
        if (on)
            set(false);
        below_on_since = above_off_since = 0;
    }
};

// ---------- MQTT callbacks ----------

// Re-subscribe on every (re)connect. With clean_session=true the broker drops
// our subscriptions on disconnect, so without this a broker restart or network
// blip silently kills heater control: publishing keeps working but no temp
// messages arrive, the temps go stale, and all heaters get forced off.
static void on_mqtt_connect(mosquitto *mosq, void *, int rc)
{
    if (rc != 0)
    {
        fprintf(stderr, "MQTT on_connect: rc=%d\n", rc);
        return;
    }
    print_timestamp();
    printf("MQTT connected — subscribing to boiler temperatures\n");
    mosquitto_subscribe(mosq, nullptr, "home/boiler/temp_hot_water", /*qos=*/0);
    mosquitto_subscribe(mosq, nullptr, "home/boiler/temp_upper_buf", /*qos=*/0);
}

static void on_mqtt_message(mosquitto *, void *, const mosquitto_message *msg)
{
    if (msg->payloadlen <= 0)
        return;
    float temp;
    if (sscanf(static_cast<const char *>(msg->payload), "%f", &temp) != 1)
        return;
    time_t now = time(nullptr);
    if (strcmp(msg->topic, "home/boiler/temp_hot_water") == 0)
    {
        g_hot_water_temp.store(temp);
        g_hot_water_temp_seen_at.store(now);
    }
    else if (strcmp(msg->topic, "home/boiler/temp_upper_buf") == 0)
    {
        g_buffer_upper_temp.store(temp);
        g_buffer_upper_temp_seen_at.store(now);
    }
}

// ---------- Main ----------

int main()
{
    setvbuf(stdout, nullptr, _IOLBF, 0); // flush log on every newline
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // Modbus setup
    modbus_t *ctx = modbus_new_rtu(SERIAL_PORT, MODBUS_BAUD,
                                   MODBUS_PARITY, MODBUS_DATA_BITS, MODBUS_STOP_BITS);
    if (!ctx)
    {
        fprintf(stderr, "Failed to create Modbus context\n");
        return 1;
    }
    modbus_set_slave(ctx, METER_SLAVE_ADDR);
    modbus_set_response_timeout(ctx, 1, 0); // 1 second

    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Modbus connect failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 1;
    }
    // Clear any stale bytes in the kernel's serial buffer from previous sessions.
    // Without this, a leftover byte from a prior run can confuse the first request.
    modbus_flush(ctx);

    // MQTT setup
    mosquitto_lib_init();
    mosquitto *mosq = mosquitto_new(MQTT_CLIENT_ID, /*clean_session=*/true, nullptr);
    if (!mosq)
    {
        fprintf(stderr, "Failed to create mosquitto client\n");
        modbus_close(ctx);
        modbus_free(ctx);
        mosquitto_lib_cleanup();
        return 1;
    }
    // Register callbacks before connecting so on_connect fires for the initial
    // CONNACK too (and re-subscribes us on every later reconnect).
    mosquitto_connect_callback_set(mosq, on_mqtt_connect);
    mosquitto_message_callback_set(mosq, on_mqtt_message);

    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS)
    {
        fprintf(stderr, "MQTT connect to %s:%d failed. Is mosquitto running?\n",
                MQTT_HOST, MQTT_PORT);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        modbus_close(ctx);
        modbus_free(ctx);
        return 1;
    }

    // Background network thread — handles keepalives/reconnects.
    // on_mqtt_connect re-subscribes to boiler temps on every (re)connect.
    mosquitto_loop_start(mosq);

    printf("Polling SDM630 at %s every %d seconds. Publishing to %s://%s:%d under %s/*\n",
           SERIAL_PORT, POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_TOPIC_PREFIX);
    printf("Ctrl-C to stop.\n");

    HeaterController heater_l1 {"L1",  HEATER_L1_GPIO_LINE,  HOT_WATER_MAX_TEMP, HEATER_LOAD_L1_W};
    HeaterController heater_l1b{"L1b", HEATER_L1B_GPIO_LINE, BUFFER_MAX_TEMP,    HEATER_LOAD_L1B_W};
    HeaterController heater_l2 {"L2",  HEATER_L2_GPIO_LINE,  BUFFER_MAX_TEMP,    HEATER_LOAD_L2_W};
    HeaterController heater_l3 {"L3",  HEATER_L3_GPIO_LINE,  BUFFER_MAX_TEMP,    HEATER_LOAD_L3_W};

    printf("Break-even thresholds (pellets %.2f c/kWh, export %.2f c/kWh, "
           "grid cheap %.2f / peak %.2f c/kWh):\n",
           PELLET_COST_C, EXPORT_RATE_C, GRID_CHEAP_C, GRID_PEAK_C);
    heater_l1.log_thresholds();
    heater_l1b.log_thresholds();
    heater_l2.log_thresholds();
    heater_l3.log_thresholds();
    printf("\n");

    if (!heater_l1.init())
        fprintf(stderr, "L1 heater control disabled — monitor-only for that phase\n");
    if (!heater_l1b.init())
        fprintf(stderr, "L1b heater control disabled — spillover unavailable\n");
    if (!heater_l2.init())
        fprintf(stderr, "L2 heater control disabled — monitor-only for that phase\n");
    if (!heater_l3.init())
        fprintf(stderr, "L3 heater control disabled — monitor-only for that phase\n");

    // Main loop
    int consecutive_failures = 0;
    constexpr int RECONNECT_AFTER_FAILURES = 3; // close+reopen port after this many
    constexpr int MAX_BACKOFF_SEC = 30;         // cap the wait between retries

    // -1 until the first cycle, so the window in force at startup is logged
    // too. Logging each crossing makes the 06:00 and 16:00 behaviour readable
    // from the journal: running elements can drop out ~60 s later as both
    // pivots tighten. Per-element values are in the startup table above.
    int last_cheap_window = -1;

    // Latches the meter-failure hold so entering and leaving it are logged
    // once each, not on every cycle of the outage.
    bool heaters_held_for_meter = false;

    while (g_running)
    {
        uint16_t block1[BLOCK1_COUNT];
        uint16_t total_p_regs[TOTAL_P_COUNT];

        int rc1 = modbus_read_input_registers(ctx, BLOCK1_START, BLOCK1_COUNT, block1);
        int rc2 = (rc1 == -1) ? -1 : modbus_read_input_registers(ctx, TOTAL_P_START, TOTAL_P_COUNT, total_p_regs);

        if (rc1 == -1 || rc2 == -1)
        {
            consecutive_failures++;
            print_timestamp();
            printf("Modbus read failed (#%d): %s\n",
                   consecutive_failures, modbus_strerror(errno));

            // Drop the heaters before attempting recovery or sleeping, so
            // the hold takes effect immediately rather than after a 30 s
            // backoff. Logged once on entry, not every cycle.
            if (consecutive_failures >= METER_FAILURE_LIMIT)
            {
                if (!heaters_held_for_meter)
                {
                    print_timestamp();
                    printf("Meter unreadable for %d consecutive reads — forcing all "
                           "heaters OFF and holding until reads resume\n",
                           consecutive_failures);
                    heaters_held_for_meter = true;
                }
                time_t fail_now = time(nullptr);
                float hw_t = g_hot_water_temp.load();
                time_t hw_seen = g_hot_water_temp_seen_at.load();
                float bu_t = g_buffer_upper_temp.load();
                time_t bu_seen = g_buffer_upper_temp_seen_at.load();
                heater_l1.meter_failure_hold(hw_t, hw_seen, fail_now);
                heater_l1b.meter_failure_hold(bu_t, bu_seen, fail_now);
                heater_l2.meter_failure_hold(bu_t, bu_seen, fail_now);
                heater_l3.meter_failure_hold(bu_t, bu_seen, fail_now);
            }

            // After a few failures, fully reset the serial port — closes the
            // file descriptor and reopens it. This clears any stuck state on
            // both the kernel side and any partial frames in-flight.
            if (consecutive_failures % RECONNECT_AFTER_FAILURES == 0)
            {
                print_timestamp();
                printf("Reopening serial port to recover...\n");
                modbus_close(ctx);
                if (modbus_connect(ctx) == -1)
                {
                    print_timestamp();
                    printf("Reconnect failed: %s\n", modbus_strerror(errno));
                }
                else
                {
                    modbus_flush(ctx);
                }
            }

            // Exponential backoff: 5s, 10s, 20s, 30s, 30s, 30s...
            int wait = POLL_INTERVAL_SEC * (1 << (consecutive_failures - 1));
            if (wait > MAX_BACKOFF_SEC)
                wait = MAX_BACKOFF_SEC;
            sleep(wait);
            continue;
        }

        // Successful read — announce recovery if we were in an outage
        if (consecutive_failures > 0)
        {
            print_timestamp();
            printf("Recovered after %d failure(s)\n", consecutive_failures);
            if (heaters_held_for_meter)
            {
                print_timestamp();
                printf("Meter readable again — heater hold released; heaters must "
                       "re-qualify through the normal %d s turn-on path\n",
                       TURN_ON_DELAY_SEC);
                heaters_held_for_meter = false;
            }
            consecutive_failures = 0;
        }

        // Decode — each float lives in 2 consecutive registers, high word first
        float v_l1 = regs_to_float(block1[0], block1[1]);
        float v_l2 = regs_to_float(block1[2], block1[3]);
        float v_l3 = regs_to_float(block1[4], block1[5]);
        float i_l1 = regs_to_float(block1[6], block1[7]);
        float i_l2 = regs_to_float(block1[8], block1[9]);
        float i_l3 = regs_to_float(block1[10], block1[11]);
        float p_l1 = regs_to_float(block1[12], block1[13]);
        float p_l2 = regs_to_float(block1[14], block1[15]);
        float p_l3 = regs_to_float(block1[16], block1[17]);
        float p_total = regs_to_float(total_p_regs[0], total_p_regs[1]);

        // Print a one-line summary
        print_timestamp();
        printf("V: %.1f/%.1f/%.1f  I: %.2f/%.2f/%.2f  "
               "P: %.0f/%.0f/%.0f W  Total: %.0f W %s\n",
               v_l1, v_l2, v_l3,
               i_l1, i_l2, i_l3,
               p_l1, p_l2, p_l3,
               p_total, (p_total < 0) ? "(exporting)" : "(importing)");

        // Publish to MQTT
        publish_float(mosq, "voltage_L1", v_l1);
        publish_float(mosq, "voltage_L2", v_l2);
        publish_float(mosq, "voltage_L3", v_l3);
        publish_float(mosq, "current_L1", i_l1);
        publish_float(mosq, "current_L2", i_l2);
        publish_float(mosq, "current_L3", i_l3);
        publish_float(mosq, "power_L1", p_l1);
        publish_float(mosq, "power_L2", p_l2);
        publish_float(mosq, "power_L3", p_l3);
        publish_float(mosq, "power_total", p_total);

        time_t now = time(nullptr);

        const int cheap_now = is_cheap_window(now) ? 1 : 0;
        if (cheap_now != last_cheap_window)
        {
            print_timestamp();
            printf("Tariff window: %s (%.2f c/kWh, solar share needed %.0f%%)\n",
                   cheap_now ? "CHEAP" : "PEAK",
                   cheap_now ? GRID_CHEAP_C : GRID_PEAK_C,
                   100.0f * (cheap_now ? SOLAR_FRAC_CHEAP : SOLAR_FRAC_PEAK));
            last_cheap_window = cheap_now;
        }

        float hw_t = g_hot_water_temp.load();
        time_t hw_seen = g_hot_water_temp_seen_at.load();
        float bu_t = g_buffer_upper_temp.load();
        time_t bu_seen = g_buffer_upper_temp_seen_at.load();

        heater_l1.update(p_l1, hw_t, hw_seen, now);
        heater_l2.update(p_l2, bu_t, bu_seen, now);
        heater_l3.update(p_l3, bu_t, bu_seen, now);

        // L1 spillover: only allowed to consider running when the hot water tank
        // is known-fresh AND at/above its cap. Otherwise the L1 primary owns the
        // phase's surplus, and L1b stays off so the two don't compete for it.
        bool hw_fresh = (hw_seen != 0 && (now - hw_seen) < TEMP_STALE_SEC);
        bool hw_at_cap = hw_fresh && hw_t >= HOT_WATER_MAX_TEMP;
        if (hw_at_cap)
            heater_l1b.update(p_l1, bu_t, bu_seen, now);
        else
            heater_l1b.force_off();

        // Publish per-heater state for monitoring
        const HeaterController *heaters[4] = {&heater_l1, &heater_l1b, &heater_l2, &heater_l3};
        for (const HeaterController *h : heaters)
        {
            char hpayload[2] = {h->on ? '1' : '0', 0};
            char htopic[128];
            // h->name is "L1"/"L1b"/"L2"/"L3"; publish lowercase
            // ("heater_l1_state", "heater_l1b_state") for consistency with
            // the rest of home/solar/*.
            char name_lc[8] = {0};
            for (size_t j = 0; j + 1 < sizeof(name_lc) && h->name[j]; j++)
                name_lc[j] = static_cast<char>(
                    tolower(static_cast<unsigned char>(h->name[j])));
            snprintf(htopic, sizeof(htopic), "%s/heater_%s_state",
                     MQTT_TOPIC_PREFIX, name_lc);
            mosquitto_publish(mosq, nullptr, htopic, 1, hpayload, 0, false);
        }

        sleep(POLL_INTERVAL_SEC);
    }

    // Cleanup on graceful exit (SIGTERM from systemd, or Ctrl-C).
    //
    // Heaters go first, before anything else can fail: each shutdown() drives
    // its line to logical OFF and only then releases it, which is what leaves
    // the relay module idle rather than latched on. Nothing below this point
    // is allowed to run before the heaters are safe.
    printf("\nShutting down...\n");
    HeaterController *all_heaters[4] = {&heater_l1, &heater_l1b, &heater_l2, &heater_l3};
    for (HeaterController *h : all_heaters)
    {
        h->force_off(); // logs the OFF transition if it was running
        h->shutdown();  // drive logical OFF, then release the line
    }
    printf("All heater lines driven OFF and released.\n");
    mosquitto_loop_stop(mosq, /*force=*/true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
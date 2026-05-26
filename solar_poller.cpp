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

// Three single-phase 1.2 kW resistive elements, one per phase, each switched
// by its own SSR. GPIO line numbers refer to gpiochip0 offsets.
//   L1: PA9, physical pin 33 — domestic hot water tank
//   L2: PA8, physical pin 31 — accumulator (buffer) tank
//   L3: PA7, physical pin 29 — accumulator (buffer) tank
constexpr unsigned int HEATER_L1_GPIO_LINE = 9;
constexpr unsigned int HEATER_L2_GPIO_LINE = 8;
constexpr unsigned int HEATER_L3_GPIO_LINE = 7;
constexpr float HEATER_LOAD_W = 1200.0f;

// Per-phase decision: each heater is judged on the export of its own phase.
// Utility meters per-phase, so a phase has to export enough on its own to
// absorb its heater. Hysteresis timers prevent rapid cycling.
constexpr float HOT_WATER_MAX_TEMP = 70.0f;             // °C — L1 (hot water tank) cap
constexpr float BUFFER_MAX_TEMP = 85.0f;                // °C — L2/L3 (accumulator) cap
constexpr float TURN_ON_THRESHOLD_W = -(HEATER_LOAD_W); // -1200 W
constexpr float TURN_OFF_THRESHOLD_W = -100.0f;
constexpr int TURN_ON_DELAY_SEC = 30;
constexpr int TURN_OFF_DELAY_SEC = 60;

// If no boiler temperature update arrives within this window, treat the
// last value as untrusted and force the affected heater(s) off. boiler_poller
// publishes every 60s, so 300s = 4 missed cycles of headroom.
constexpr int TEMP_STALE_SEC = 300;

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

// ---------- Heater controller ----------
//
// One instance per heater. Owns a libgpiod v2 line request on a single
// gpiochip0 offset and runs an on/off state machine driven by per-phase
// export power and a tank temperature cap. On clean shutdown (or crash →
// systemd cleans up the FD), the line request is released, the kernel
// returns the line to its default state, and the SSR input is no longer
// driven → heater off (the safe default).
struct HeaterController
{
    const char *name;       // "L1" / "L2" / "L3" — used in log lines
    unsigned int gpio_line; // gpiochip0 offset for this heater's SSR
    float temp_cap;         // °C — refuse to heat at or above this

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

    // p_decision_w: per-phase power (negative = export) for this heater's phase.
    // tank_temp: latest tank temperature reading (°C) for the tank this heater
    // sits in. Heater will not turn on (and will turn off) if temp is at or
    // above temp_cap.
    // tank_temp_seen_at: wall-clock time of the most recent temp message
    // (0 = never received). If older than TEMP_STALE_SEC, the heater is
    // forced off — running on a stale reading risks overshooting the cap.
    void update(float p_decision_w, float tank_temp,
                time_t tank_temp_seen_at, time_t now)
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
            return;
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
            return;
        }
        if (!on)
        {
            if (!request)
                return;
            if (p_decision_w < TURN_ON_THRESHOLD_W)
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
            if (p_decision_w > TURN_OFF_THRESHOLD_W)
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

    void shutdown()
    {
        if (request)
        {
            gpiod_line_request_set_value(request, gpio_line, GPIOD_LINE_VALUE_INACTIVE);
            gpiod_line_request_release(request);
            request = nullptr;
        }
        if (chip)
        {
            gpiod_chip_close(chip);
            chip = nullptr;
        }
    }
};

// ---------- MQTT message callback ----------

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
    // Subscribe to boiler temperatures for per-heater cap logic
    mosquitto_message_callback_set(mosq, on_mqtt_message);
    mosquitto_subscribe(mosq, nullptr, "home/boiler/temp_hot_water", /*qos=*/0);
    mosquitto_subscribe(mosq, nullptr, "home/boiler/temp_upper_buf", /*qos=*/0);

    // Background network thread — handles keepalives/reconnects
    mosquitto_loop_start(mosq);

    printf("Polling SDM630 at %s every %d seconds. Publishing to %s://%s:%d under %s/*\n",
           SERIAL_PORT, POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_TOPIC_PREFIX);
    printf("Ctrl-C to stop.\n\n");

    HeaterController heater_l1{"L1", HEATER_L1_GPIO_LINE, HOT_WATER_MAX_TEMP};
    HeaterController heater_l2{"L2", HEATER_L2_GPIO_LINE, BUFFER_MAX_TEMP};
    HeaterController heater_l3{"L3", HEATER_L3_GPIO_LINE, BUFFER_MAX_TEMP};
    if (!heater_l1.init())
        fprintf(stderr, "L1 heater control disabled — monitor-only for that phase\n");
    if (!heater_l2.init())
        fprintf(stderr, "L2 heater control disabled — monitor-only for that phase\n");
    if (!heater_l3.init())
        fprintf(stderr, "L3 heater control disabled — monitor-only for that phase\n");

    // Main loop
    int consecutive_failures = 0;
    constexpr int RECONNECT_AFTER_FAILURES = 3; // close+reopen port after this many
    constexpr int MAX_BACKOFF_SEC = 30;         // cap the wait between retries

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
        float hw_t = g_hot_water_temp.load();
        time_t hw_seen = g_hot_water_temp_seen_at.load();
        float bu_t = g_buffer_upper_temp.load();
        time_t bu_seen = g_buffer_upper_temp_seen_at.load();
        heater_l1.update(p_l1, hw_t, hw_seen, now);
        heater_l2.update(p_l2, bu_t, bu_seen, now);
        heater_l3.update(p_l3, bu_t, bu_seen, now);

        // Publish per-heater state for monitoring
        const HeaterController *heaters[3] = {&heater_l1, &heater_l2, &heater_l3};
        for (const HeaterController *h : heaters)
        {
            char hpayload[2] = {h->on ? '1' : '0', 0};
            char htopic[128];
            // h->name is "L1"/"L2"/"L3"; publish lowercase ("heater_l1_state")
            // for consistency with the rest of home/solar/*.
            snprintf(htopic, sizeof(htopic), "%s/heater_%c%c_state",
                     MQTT_TOPIC_PREFIX,
                     static_cast<char>(tolower(h->name[0])), h->name[1]);
            mosquitto_publish(mosq, nullptr, htopic, 1, hpayload, 0, false);
        }

        sleep(POLL_INTERVAL_SEC);
    }

    // Cleanup on graceful exit
    printf("\nShutting down...\n");
    heater_l1.shutdown();
    heater_l2.shutdown();
    heater_l3.shutdown();
    mosquitto_loop_stop(mosq, /*force=*/true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
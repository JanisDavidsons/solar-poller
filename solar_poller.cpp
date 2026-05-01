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
constexpr unsigned int HEATER_GPIO_LINE = 9; // PA9 = physical pin 22
constexpr float HEATER_LOAD_W = 1300.0f;     // 1.3 kW resistive element

// Decision is made on phase 1 power because the heater is wired to L1.
// Negative = export. Turn on when L1 export comfortably exceeds the
// heater's draw; turn off when export drops near zero. Hysteresis timers
// prevent rapid cycling around the threshold.
//
// To switch to total-power decision (if utility does aggregated netting),
// just pass p_total to heater.update() instead of p_l1 below — thresholds
// stay the same.
constexpr float HOT_WATER_MAX_TEMP = 70.0f;             // °C — do not heat above this
constexpr float TURN_ON_THRESHOLD_W = -(HEATER_LOAD_W); // -1300 W
constexpr float TURN_OFF_THRESHOLD_W = -100.0f;
constexpr int TURN_ON_DELAY_SEC = 30;
constexpr int TURN_OFF_DELAY_SEC = 60;

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

// Latest hot water tank temperature received from home/boiler/temp_cwu (via boiler_poller).
// Written by the mosquitto network thread, read by the main loop.
static std::atomic<float> g_hot_water_temp{0.0f};

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
// Owns the libgpiod v2 line request and runs the on/off state machine.
// On clean shutdown (or crash → systemd cleans up the FD), the line request
// is released, the kernel returns line 9 to its default state, and the SSR
// input is no longer driven → heater off (the safe default).
struct HeaterController
{
    gpiod_chip *chip = nullptr;
    gpiod_line_request *request = nullptr;
    bool on = false;
    time_t below_on_since = 0;
    time_t above_off_since = 0;

    bool init()
    {
        chip = gpiod_chip_open(GPIO_CHIP_PATH);
        if (!chip)
        {
            fprintf(stderr, "gpiod_chip_open(%s) failed\n", GPIO_CHIP_PATH);
            return false;
        }

        gpiod_line_settings *settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config *line_cfg = gpiod_line_config_new();
        unsigned int offsets[1] = {HEATER_GPIO_LINE};
        gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);

        gpiod_request_config *req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "solar_poller");

        request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);

        if (!request)
        {
            fprintf(stderr, "gpiod_chip_request_lines failed: %s\n", strerror(errno));
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
        gpiod_line_request_set_value(request, HEATER_GPIO_LINE,
                                     turn_on ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
        on = turn_on;
        print_timestamp();
        printf("Heater %s\n", turn_on ? "ON" : "OFF");
    }

    // p_decision_w: the power reading the state machine evaluates.
    // Pass p_l1 for per-phase logic, p_total for aggregated logic.
    // hot_water_temp: latest hot water tank temperature (°C). Heater will not
    // turn on (and will turn off) if temp is at or above HOT_WATER_MAX_TEMP.
    void update(float p_decision_w, float hot_water_temp, time_t now)
    {
        if (hot_water_temp >= HOT_WATER_MAX_TEMP)
        {
            if (on)
            {
                print_timestamp();
                printf("Heater OFF — hot water at %.1f°C (limit %.0f°C)\n",
                       hot_water_temp, HOT_WATER_MAX_TEMP);
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
            gpiod_line_request_set_value(request, HEATER_GPIO_LINE, GPIOD_LINE_VALUE_INACTIVE);
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
    if (msg->payloadlen > 0 &&
        strcmp(msg->topic, "home/boiler/temp_hot_water") == 0)
    {
        float temp;
        if (sscanf(static_cast<const char *>(msg->payload), "%f", &temp) == 1)
            g_hot_water_temp.store(temp);
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
    // Subscribe to boiler temperature for heater cap logic
    mosquitto_message_callback_set(mosq, on_mqtt_message);
    mosquitto_subscribe(mosq, nullptr, "home/boiler/temp_hot_water", /*qos=*/0);

    // Background network thread — handles keepalives/reconnects
    mosquitto_loop_start(mosq);

    printf("Polling SDM630 at %s every %d seconds. Publishing to %s://%s:%d under %s/*\n",
           SERIAL_PORT, POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_TOPIC_PREFIX);
    printf("Ctrl-C to stop.\n\n");

    HeaterController heater;
    if (!heater.init())
    {
        fprintf(stderr, "Heater control disabled — continuing in monitor-only mode\n");
    }

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

        heater.update(p_l1, g_hot_water_temp.load(), time(nullptr));

        // Publish heater state for monitoring
        char hpayload[2] = {heater.on ? '1' : '0', 0};
        char htopic[128];
        snprintf(htopic, sizeof(htopic), "%s/heater_state", MQTT_TOPIC_PREFIX);
        mosquitto_publish(mosq, nullptr, htopic, 1, hpayload, 0, false);

        sleep(POLL_INTERVAL_SEC);
    }

    // Cleanup on graceful exit
    printf("\nShutting down...\n");
    heater.shutdown();
    mosquitto_loop_stop(mosq, /*force=*/true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
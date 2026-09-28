#!/usr/bin/env python3
"""
boiler_poller.py

Polls the Plum ecoNET 300 module on the LAN for boiler/buffer/CWU
temperatures every 60 seconds and publishes them to MQTT. Intended to
run as a systemd service alongside solar_poller.

Reads directly from the controller's local web interface
(http://<ip>/econet/regParams) using HTTP Basic Auth. No internet
dependency, no rotating session cookies.

Topics published (under home/boiler/):
  temp_hot_water   — domestic hot water tank temp (°C)
  temp_cwu_set     — pellet burner CWU target (°C)
  temp_upper_buf   — top of buffer tank (°C)
  temp_lower_buf   — bottom of buffer tank (°C)
  temp_co          — central heating water (°C)
  fuel_level       — pellet hopper level (0-9?)
  mode             — burner mode code
  online           — 1 if last fetch succeeded, 0 if not

The MQTT messages are published with `retain=True` so a fresh subscriber
(like solar_poller after a restart) immediately gets the last known value
without waiting up to 60s for the next poll.
"""

import json
import logging
import os
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt
import requests

# ---------- Config ----------
# Plum ecoNET 300 module on the LAN. Same data the cloud sees, but no
# auth dance: just HTTP Basic with the module's web UI credentials.
ECONET_URL  = "http://192.168.8.3/econet/regParams"
ECONET_AUTH = ("admin", "admin")

# Editable service parameters live in a separate endpoint, as a flat list
# indexed by position. Indices come from rmParamsNames (Latvian UI strings)
# and are firmware-dependent, confirmed 2026-09-25 on ecoNET 300 v3.0.023:
#   160 "Akumulācij.vadība"      buffer control on/off
#   161 "Akumul. uzsildes temp."  buffer loading start (°C)
#   162 "Akumul. augšējā temp."   buffer loading end (°C)
#   181 "Režģa režīms"            grate (wood) mode on/off, 1 = wood
#   183 "Automātiska pāreja uz granulām"  auto switch back to pellets
#    53 "Uzstādītā Katla temperatūra"  boiler setpoint, pellet mode
#   184 "Uzstādītā Katla temperatūra"  boiler setpoint, grate mode
# 181 is the user's SELECTION, not the fuel actually burning: on the
# automatic wood->pellet takeover (2026-09-28 01:20) it stayed at 1. The
# controller does expose the active fuel indirectly: regParams tempCOSet
# equals whichever of the two stored setpoints (53 / 184) is active. That
# comparison drives `fuel_type` below.
# Read every poll: grate mode is toggled by hand and we want the timeline
# to show the change within a minute.
PARAMS_URL = "http://192.168.8.3/econet/rmParamsData"
PARAM_MAP = {
    161: "buffer_start_set",
    162: "buffer_end_set",
    181: "grate_mode",
    183: "auto_pellet_switch",
    53:  "temp_co_set_pellets",
    184: "temp_co_set_wood",
    48:  "feeder_rate_kg_h",
}
PARAMS_EVERY_N_POLLS = 1

# Pellet consumption. The local module exposes no fuel counter (fuelConsum
# exists only in the econet24 cloud payload), so we reproduce the
# controller's own arithmetic: feeder on-time x feeder capacity (param 48
# "Padevēja veiktspēja", kg/h, 11.4 on 2026-09-28). A second thread samples
# regParams every FEEDER_SAMPLE_SEC and accumulates seconds with
# feederWorks == true. Counters are persisted so a restart does not reset
# a day. Integrating the once-a-minute `fuel_stream` undercounted ~10%.
FEEDER_SAMPLE_SEC = 5
COUNTER_FILE = "/var/lib/boiler_poller/counters.json"
# Set from a hopper-weighing calibration once done; 1.0 = trust param 48.
PELLET_CALIBRATION = 1.0

# fuel_type codes published on home/boiler/fuel_type
FUEL_WOOD = 0
FUEL_PELLETS = 1

MQTT_HOST   = "localhost"
MQTT_PORT   = 1883
MQTT_PREFIX = "home/boiler"
POLL_INTERVAL_SEC = 60
HTTP_TIMEOUT_SEC  = 15

# Map of (econet curr field) -> (mqtt sub-topic)
# Add or remove fields here as needs evolve.
FIELD_MAP = {
    "tempCWU":         "temp_hot_water",
    "tempCWUSet":      "temp_cwu_set",
    "tempUpperBuffer": "temp_upper_buf",
    "tempLowerBuffer": "temp_lower_buf",
    "tempCO":          "temp_co",
    "tempCOSet":       "temp_co_set",
    "fuelLevel":       "fuel_level",
    "fuelStream":      "fuel_stream",
    "boilerPower":     "boiler_power",
    "boilerPowerKW":   "boiler_power_kw",
    "fanPower":        "fan_power",
    "tempFeeder":      "temp_feeder",
    "tempFlueGas":     "temp_flue_gas",
    "feederWorks":     "feeder_works",
    "fanWorks":        "fan_works",
    # Pump run states (bool, 0/1). Added 2026-09-28 to find out what drives
    # the loading pump while the burner is Halted: with the LK 820 bypass
    # throttled since 09-27, a pump run pulls buffer-bottom water through
    # the idle boiler, drops it ~8 C in minutes, and the controller relights
    # on boiler hysteresis, giving 10-15 min burns with no benefit to the
    # buffer. pumpCO/pumpCWU without "Works" only say the pump is configured.
    "pumpCOWorks":     "pump_co_works",
    "pumpCWUWorks":    "pump_cwu_works",
    "pumpCirculationWorks": "pump_circ_works",
    "mode":            "mode",
}

# Fields solar_poller relies on for heater control. If any of these come
# back null (sensor unplugged / controller in a state that doesn't report
# them), the heater stack will stale-out and force every element off — so
# treat the fetch as a failure even though the HTTP layer succeeded.
CRITICAL_FIELDS = ("tempCWU", "tempUpperBuffer")

# ---------- Setup ----------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("boiler_poller")

running = True
def on_signal(signum, frame):
    global running
    log.info("Caught signal %d, shutting down", signum)
    running = False
signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)

# ---------- MQTT ----------
mqttc = mqtt.Client(client_id="boiler_poller")
mqttc.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqttc.loop_start()

def publish(sub_topic, value):
    """Publish with retain=True so subscribers see the latest value immediately."""
    topic = f"{MQTT_PREFIX}/{sub_topic}"
    if isinstance(value, bool):
        payload = "1" if value else "0"   # Telegraf parses bare numbers only
    elif isinstance(value, float):
        payload = f"{value:.2f}"
    else:
        payload = str(value)
    mqttc.publish(topic, payload, qos=0, retain=True)

# ---------- Poll loop ----------
def fetch_once():
    """One HTTP fetch + parse. Returns dict of curr fields, or None on failure."""
    try:
        resp = requests.get(
            ECONET_URL,
            auth=ECONET_AUTH,
            timeout=HTTP_TIMEOUT_SEC,
        )
        resp.raise_for_status()
        return resp.json().get("curr", {})
    except requests.RequestException as e:
        log.warning("HTTP fetch failed: %s", e)
        return None
    except (ValueError, KeyError) as e:
        log.warning("JSON parse failed: %s", e)
        return None

def fetch_params():
    """Fetch rmParamsData and return {sub_topic: value} for PARAM_MAP, or None."""
    try:
        resp = requests.get(PARAMS_URL, auth=ECONET_AUTH, timeout=HTTP_TIMEOUT_SEC)
        resp.raise_for_status()
        data = resp.json()["data"]
        return {topic: data[idx]["value"] for idx, topic in PARAM_MAP.items()}
    except requests.RequestException as e:
        log.warning("Params fetch failed: %s", e)
        return None
    except (ValueError, KeyError, IndexError, TypeError) as e:
        log.warning("Params parse failed (index shift after firmware update?): %s", e)
        return None

log.info("Polling %s every %ds, publishing to %s://%s:%d under %s/*",
         ECONET_URL, POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_PREFIX)

# ---------- Feeder on-time counter (background thread) ----------
counter_lock = threading.Lock()
counters = {"feeder_seconds_total": 0.0}

def load_counters():
    try:
        with open(COUNTER_FILE) as f:
            saved = json.load(f)
        counters["feeder_seconds_total"] = float(saved.get("feeder_seconds_total", 0.0))
        log.info("Loaded counters: feeder_seconds_total=%.0f", counters["feeder_seconds_total"])
    except FileNotFoundError:
        log.info("No counter file yet, starting from zero")
    except (ValueError, OSError) as e:
        log.warning("Counter file unreadable, starting from zero: %s", e)

def save_counters():
    try:
        os.makedirs(os.path.dirname(COUNTER_FILE), exist_ok=True)
        tmp = COUNTER_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(counters, f)
        os.replace(tmp, COUNTER_FILE)
    except OSError as e:
        log.warning("Could not save counters: %s", e)

def feeder_sampler():
    """Sample feederWorks every FEEDER_SAMPLE_SEC and accumulate on-time.
    Credits the elapsed interval when the feeder is seen running, so the
    error per feed pulse is at most one sample interval."""
    last_t = time.monotonic()
    while running:
        time.sleep(FEEDER_SAMPLE_SEC)
        now = time.monotonic()
        dt = now - last_t
        last_t = now
        try:
            resp = requests.get(ECONET_URL, auth=ECONET_AUTH, timeout=HTTP_TIMEOUT_SEC)
            resp.raise_for_status()
            works = resp.json().get("curr", {}).get("feederWorks")
        except (requests.RequestException, ValueError) as e:
            log.debug("Feeder sample failed: %s", e)
            continue
        if works:
            with counter_lock:
                counters["feeder_seconds_total"] += dt

load_counters()
threading.Thread(target=feeder_sampler, name="feeder_sampler", daemon=True).start()

def publish_counters(params):
    """Publish cumulative feeder seconds and derived pellet kg, persist to disk."""
    with counter_lock:
        secs = counters["feeder_seconds_total"]
    rate = (params or {}).get("feeder_rate_kg_h")
    publish("feeder_seconds_total", round(secs, 1))
    if rate:
        publish("pellets_kg_total", secs / 3600.0 * float(rate) * PELLET_CALIBRATION)
    save_counters()

consecutive_failures = 0
poll_count = 0
params = None
last_fuel_type = None

def derive_fuel_type(curr, params, last):
    """Active fuel from tempCOSet vs the two stored setpoints. Returns
    FUEL_WOOD / FUEL_PELLETS, or `last` when it cannot tell (both setpoints
    equal, or a field missing)."""
    if not params or curr is None:
        return last
    active = curr.get("tempCOSet")
    pellets = params.get("temp_co_set_pellets")
    wood = params.get("temp_co_set_wood")
    if active is None or pellets is None or wood is None:
        return last
    if pellets == wood:
        if last is None:
            log.warning("Pellet and wood setpoints are equal (%s); cannot derive fuel_type", pellets)
        return last
    if active == pellets:
        return FUEL_PELLETS
    if active == wood:
        return FUEL_WOOD
    log.warning("tempCOSet=%s matches neither setpoint (pellets=%s, wood=%s)", active, pellets, wood)
    return last

while running:
    # Service parameters: not critical for heater control, so a failure here
    # only logs and leaves `online` alone.
    if poll_count % PARAMS_EVERY_N_POLLS == 0:
        fetched = fetch_params()
        if fetched:
            params = fetched
            for topic, val in params.items():
                publish(topic, val)
    poll_count += 1

    curr = fetch_once()

    publish_counters(params)

    fuel_type = derive_fuel_type(curr, params, last_fuel_type)
    if fuel_type is not None:
        if fuel_type != last_fuel_type:
            log.info("fuel_type -> %s", "pellets" if fuel_type == FUEL_PELLETS else "wood")
        publish("fuel_type", fuel_type)
        last_fuel_type = fuel_type
    if curr is None:
        consecutive_failures += 1
        publish("online", 0)
        # Backoff: keep trying every minute, but log loudly after sustained failure
        if consecutive_failures == 5:
            log.error("5 consecutive fetch failures — ecoNET module unreachable "
                      "or credentials wrong (check %s)", ECONET_URL)
    else:
        # Publish whatever fields we did get — non-critical fields like
        # mode/fuel_level are still useful even when temps are missing.
        for field, sub_topic in FIELD_MAP.items():
            val = curr.get(field)
            if val is None:
                continue
            publish(sub_topic, val)

        missing = [f for f in CRITICAL_FIELDS if curr.get(f) is None]
        if missing:
            consecutive_failures += 1
            publish("online", 0)
            log.warning("Response missing critical field(s): %s — heater control will stale out",
                        ", ".join(missing))
            if consecutive_failures == 5:
                log.error("5 consecutive fetches with missing critical fields — "
                          "controller may have stopped reporting (sensor "
                          "unplugged or controller in an unusual state).")
        else:
            if consecutive_failures > 0:
                log.info("Recovered after %d failure(s)", consecutive_failures)
            consecutive_failures = 0
            publish("online", 1)

        # One-line summary log so journalctl shows a heartbeat. Render
        # missing fields as "<missing>" rather than collapsing them to 0.0,
        # which would mask the viewer-gone failure mode.
        def fmt(v):
            return f"{v:.1f}°C" if v is not None else "<missing>"
        log.info("hot_water=%s upper=%s lower=%s",
                 fmt(curr.get("tempCWU")),
                 fmt(curr.get("tempUpperBuffer")),
                 fmt(curr.get("tempLowerBuffer")))

    # Sleep in 1-second chunks so SIGTERM gets responded to within 1s
    for _ in range(POLL_INTERVAL_SEC):
        if not running:
            break
        time.sleep(1)

# ---------- Cleanup ----------
log.info("Shutting down...")
save_counters()
publish("online", 0)
mqttc.loop_stop()
mqttc.disconnect()
sys.exit(0)
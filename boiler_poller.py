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
from datetime import datetime, timezone

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
    187: "exhaust_level",       # "Vilkmes jauda - malkas režīms", % (2026-10-12)
}
PARAMS_EVERY_N_POLLS = 1

# Zero-glitch guard (2026-10-12): rmParamsData now and then answers with
# zeros. At 00:01 and 00:23 the poller read Buffer loading end as 0, published
# it and the supervisor wrote over it. Boiler setpoint in grate mode (184) is
# never 0 in a real answer, so a 0 there (or an empty or short list) marks the
# whole response as glitched: it is discarded and the previous values stay.
ZERO_GUARD_IDX = 184

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

# ---------- Supervisor: Buffer loading end by fuel (2026-09-30) ----------
# The controller has one "Buffer loading end" (162, judged on the lower
# buffer probe) for both fuels, and neither value suits both:
#   pellets 50: the charge ends on "buffer loaded" while the return is still
#     cold enough to absorb the burner's 5 kW minimum. At 62 the burn
#     overshot at end of charge (bottom ~60), stopped on boiler overheat and
#     relit on hysteresis 3.5 h later (09-30 13:00 -> 16:27). 50 also leaves
#     room in the tank for the next day's solar. The buffer stratifies
#     strongly, so DHW (fed from the top via the coil) still reaches 55+.
#   wood 72 (62 until 2026-10-12): wood cannot be modulated; stop the charge
#     too early and the fan parks while the load smoulders. With the wood
#     setpoint now 80, ending at 72 stores about 8 kWh more per load
#     (10 °C x 0.81 kWh/°C). On 10-12 with 62 the loading pump kept running
#     with the bottom at 65, so for wood this value does not stop the blower.
# The poller writes 162 when fuel_type has been stable for SETTLE polls and
# the controller's value differs from the target. A manual change to Buffer
# loading end in the UI is overwritten within minutes: change the table
# below instead, or publish an override. Writes use rmNewParam and are read
# back after 30 s (the module acknowledges writes it then drops).
# The supervisor is the ONLY writer of 162. The veto-relay panel writes 187
# only and asks for another buffer end through the override topic.
#
# Override: retained JSON on home/boiler/cmd/buffer_end,
#   {"value": 66, "until": "2026-10-12T22:00:00+03:00"}
# While `until` is in the future the supervisor's target is `value` instead
# of the fuel table, written without waiting for fuel_type to settle. An
# `until` without a zone is the Pi's local time. An empty retained message
# clears the override; an expired one is ignored. Published: every poll
# buffer_end_target (what the supervisor enforces now), and on each
# confirmed write buffer_end_overwritten (the value it replaced; not
# retained, so a Telegraf reconnect does not replay it as a new event).
SUPERVISOR_ENABLED = True
WRITE_URL = "http://192.168.8.3/econet/rmNewParam"
WRITE_READBACK_SEC = 30
BUFFER_END_IDX = 162
BUFFER_END_BY_FUEL = {FUEL_PELLETS: 50, FUEL_WOOD: 72}
SUPERVISOR_SETTLE_POLLS = 3        # fuel_type must hold this many polls
SUPERVISOR_RETRY_SEC = 1800        # after a failed write, wait before retrying
CMD_BUFFER_END_TOPIC = "home/boiler/cmd/buffer_end"

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
    # Mixer circuit pumps (MIX1 floor, MIX3 radiators). Added 2026-09-29 to
    # see how often the controller runs them: in grate mode it seems to use
    # the house circuits as overheat protection, and even with the room
    # thermostats satisfied the pumps run in short bursts.
    "mixerPumpWorks1": "pump_mix1_works",
    "mixerPumpWorks3": "pump_mix3_works",
    # Mixer supply temperatures and setpoints (2026-09-30): the pump rows
    # say "running", these say with what water. MIX3 ran 35 % of the time
    # with a 62 C buffer and 10 % with a 46 C buffer; radiators were hot in
    # the first case and cold in the second.
    "mixerTemp1":      "temp_mix1",
    "mixerTemp3":      "temp_mix3",
    "mixerSetTemp1":   "temp_mix1_set",
    "mixerSetTemp3":   "temp_mix3_set",
    # ecoSTER room panels (2026-09-30): room temperature, setpoint and the
    # thermostat contact state (False = satisfied, contact open). Zone 1 is
    # the floor (MIX1), zone 3 the radiators (MIX3). Basis for a future
    # pump-veto relay: the controller kicks the mixer pumps every 5-20 min
    # even with both rooms above setpoint, which with a hot buffer heats
    # the house instead of storing wood heat.
    "ecoSterTemp1":    "room_temp1",
    "ecoSterTemp3":    "room_temp3",
    "ecoSterSetTemp1": "room_set1",
    "ecoSterSetTemp3": "room_set3",
    "ecoSterContacts1": "room_demand1",
    "ecoSterContacts3": "room_demand3",
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

# ---------- Buffer end override (set from the MQTT thread) ----------
override_lock = threading.Lock()
buffer_end_override = None          # (value, until as aware datetime) or None
buffer_end_limits = (None, None)    # (minv, maxv) of 162, from rmParamsData
supervisor_last_fail = float("-inf")

def parse_until(text):
    """ISO time -> aware datetime. 'Z' accepted; no zone = the Pi's local time."""
    text = text.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    dt = datetime.fromisoformat(text)
    if dt.tzinfo is None:
        dt = dt.astimezone()
    return dt

def on_cmd_buffer_end(client, userdata, msg):
    global buffer_end_override, supervisor_last_fail
    payload = msg.payload.decode("utf-8", "replace").strip()
    if not payload:
        with override_lock:
            buffer_end_override = None
        log.info("Buffer end override cleared")
        return
    try:
        cmd = json.loads(payload)
        value = int(cmd["value"])
        until = parse_until(str(cmd["until"]))
    except (ValueError, KeyError, TypeError) as e:
        log.warning("Ignoring %s payload %r: %s", CMD_BUFFER_END_TOPIC, payload[:200], e)
        return
    lo, hi = buffer_end_limits
    if lo is not None and hi is not None and not (lo <= value <= hi):
        log.warning("Ignoring buffer end override %s: outside %s-%s", value, lo, hi)
        return
    with override_lock:
        buffer_end_override = (value, until)
    supervisor_last_fail = float("-inf")   # act on a new command without the retry wait
    log.info("Buffer end override %s until %s", value, until.isoformat())

def active_override():
    """Override value while in force, else None. Forgets it once expired."""
    global buffer_end_override
    with override_lock:
        ov = buffer_end_override
    if ov is None:
        return None
    value, until = ov
    if datetime.now(timezone.utc) < until:
        return value
    with override_lock:
        if buffer_end_override == ov:
            buffer_end_override = None
    log.info("Buffer end override %s expired at %s", value, until.isoformat())
    return None

def on_connect(client, userdata, flags, rc, properties=None):
    # Subscribe here so a broker reconnect re-subscribes; the retained
    # command is delivered again, so an override survives a restart.
    if rc == 0:
        client.subscribe(CMD_BUFFER_END_TOPIC, qos=1)

# ---------- MQTT ----------
mqttc = mqtt.Client(client_id="boiler_poller")
mqttc.on_connect = on_connect
mqttc.message_callback_add(CMD_BUFFER_END_TOPIC, on_cmd_buffer_end)
mqttc.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqttc.loop_start()

def publish(sub_topic, value, retain=True):
    """Publish retained by default so subscribers see the latest value
    immediately. Events (buffer_end_overwritten) pass retain=False."""
    topic = f"{MQTT_PREFIX}/{sub_topic}"
    if isinstance(value, bool):
        payload = "1" if value else "0"   # Telegraf parses bare numbers only
    elif isinstance(value, float):
        payload = f"{value:.2f}"
    else:
        payload = str(value)
    mqttc.publish(topic, payload, qos=0, retain=retain)

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

def read_params_data():
    """GET rmParamsData and return its `data` list, or None on failure or
    when the zero-glitch guard rejects the answer (see ZERO_GUARD_IDX)."""
    try:
        resp = requests.get(PARAMS_URL, auth=ECONET_AUTH, timeout=HTTP_TIMEOUT_SEC)
        resp.raise_for_status()
        data = resp.json().get("data")
    except requests.RequestException as e:
        log.warning("Params fetch failed: %s", e)
        return None
    except (ValueError, AttributeError) as e:
        log.warning("Params parse failed: %s", e)
        return None
    try:
        guard = data[ZERO_GUARD_IDX]["value"]
    except (TypeError, IndexError, KeyError):
        log.warning("Params answer empty or short (%s entries), discarded",
                    len(data) if isinstance(data, list) else "no")
        return None
    if not guard:
        log.warning("Params answer has param %d = %r (zero glitch), discarded",
                    ZERO_GUARD_IDX, guard)
        return None
    return data

def fetch_params():
    """Return {sub_topic: value} for PARAM_MAP from a guarded rmParamsData
    read, or None. Also refreshes the limits of Buffer loading end."""
    global buffer_end_limits
    data = read_params_data()
    if data is None:
        return None
    try:
        out = {topic: data[idx]["value"] for idx, topic in PARAM_MAP.items()}
        lim = data[BUFFER_END_IDX]
        buffer_end_limits = (lim.get("minv"), lim.get("maxv"))
        return out
    except (KeyError, IndexError, TypeError, AttributeError) as e:
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
fuel_stable_polls = 0

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

def write_param(idx, value):
    """Write one rmParamsData entry and confirm by read-back. Returns True
    only if the controller reports the new value after WRITE_READBACK_SEC."""
    try:
        resp = requests.get(WRITE_URL,
                            params={"newParamIndex": idx, "newParamValue": value},
                            auth=ECONET_AUTH, timeout=HTTP_TIMEOUT_SEC)
        resp.raise_for_status()
        result = resp.json().get("result")
        if result != "OK":
            log.warning("Write %d=%s refused: %s", idx, value, resp.text[:200])
            return False
    except (requests.RequestException, ValueError) as e:
        log.warning("Write %d=%s failed: %s", idx, value, e)
        return False
    for _ in range(WRITE_READBACK_SEC):
        if not running:
            return False
        time.sleep(1)
    data = None
    for _ in range(3):              # a glitched read-back is retried, not taken as 0
        data = read_params_data()
        if data is not None or not running:
            break
        time.sleep(5)
    if data is None:
        log.warning("Read-back of %d failed: no valid params answer", idx)
        return False
    try:
        got = data[idx]["value"]
    except (KeyError, IndexError, TypeError) as e:
        log.warning("Read-back of %d failed: %s", idx, e)
        return False
    if got != value:
        log.warning("Write %d=%s acknowledged but read back %s", idx, value, got)
        return False
    log.info("Param %d = %s confirmed", idx, value)
    return True

def supervise_buffer_end(fuel_type, params, stable_polls):
    """Keep Buffer loading end (162) at the override value while one is in
    force, otherwise at the value for the active fuel. Only writer of 162."""
    global supervisor_last_fail
    if not SUPERVISOR_ENABLED or not params:
        return
    override = active_override()
    if override is not None:
        target, source = override, "override"
    elif fuel_type is not None:
        target = BUFFER_END_BY_FUEL.get(fuel_type)
        source = "fuel pellets" if fuel_type == FUEL_PELLETS else "fuel wood"
    else:
        return
    current = params.get("buffer_end_set")
    if target is None or current is None:
        return
    publish("buffer_end_target", target)
    if current == target:
        return
    if source != "override" and stable_polls < SUPERVISOR_SETTLE_POLLS:
        return
    if time.monotonic() - supervisor_last_fail < SUPERVISOR_RETRY_SEC:
        return
    log.info("Supervisor: Buffer loading end %s -> %s (%s)", current, target, source)
    if write_param(BUFFER_END_IDX, target):
        params["buffer_end_set"] = target
        publish("buffer_end_set", target)
        publish("buffer_end_overwritten", current, retain=False)
        supervisor_last_fail = float("-inf")
    else:
        supervisor_last_fail = time.monotonic()

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
            fuel_stable_polls = 0
        else:
            fuel_stable_polls += 1
        publish("fuel_type", fuel_type)
        last_fuel_type = fuel_type
    supervise_buffer_end(fuel_type, params, fuel_stable_polls)
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
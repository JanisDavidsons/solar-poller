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
  fuel_type        — 0 wood, 1 pellets (last known value)
  fuel_type_known  — 1 if fuel_type comes from param 182, 0 if unknown
  heat_source      — param 182 raw: 0 pellet burner, 1 wood grate, 2 backup
  flame            — optical flame sensor, % (regParams tempOpticalSensor)
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
#    68 "Katla temp. palielinājums no SKŪ un maisītāja"  boiler setpoint raise, K
#   182 "Siltuma avots"           heat source: 0 pellet burner, 1 wood grate,
#                                 2 backup boiler
# 181 is the user's SELECTION, not the fuel actually burning: on the
# automatic wood->pellet takeover (2026-09-28 01:20) it stayed at 1. 182 is
# the active heat source: 1 during the wood burn on 2026-10-14, 0 after the
# automatic switch to pellets on 10-15. fuel_type comes from 182 (see
# derive_fuel_type); the older guess from tempCOSet is only a fallback.
# 182 is editable on the controller: the poller and the panel never write it.
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
    68:  "temp_co_increase",    # input to fuel_type, see derive_fuel_type (2026-10-13)
    48:  "feeder_rate_kg_h",
    187: "exhaust_level",       # "Vilkmes jauda - malkas režīms", % (2026-10-12)
    186: "blower_level",        # "Vent. Jauda", blower % in grate mode, floor param 36 (40 since 2026-10-14); boiler-panel confirms its writes with it (2026-10-14)
    182: "heat_source",         # "Siltuma avots", source of fuel_type (2026-10-15)
    47:  "grate_fans",          # "Ventilators režģa režīmā" 0/1; 0 stops both fans, boiler-panel uses it for embers
    194: "exhaust_fan_enabled", # "Vilkmes ventilators" 0/1, set per fuel by supervise_exhaust_fan
}
PARAMS_EVERY_N_POLLS = 1

# Zero-glitch guard (2026-10-12): rmParamsData now and then answers with
# zeros. At 00:01 and 00:23 the poller read Buffer loading end as 0, published
# it and the supervisor wrote over it. The boiler setpoints (184 grate, 53
# pellets) are never 0 in a real answer, and a real answer has well over
# ZERO_GUARD_MIN_NONZERO non-zero values (173 of 206 on 2026-10-14). Failing
# any of these (or an empty or short list) marks the whole response as
# glitched: it is discarded and the previous values stay. This matters for
# 182, where 0 (pellets) is a valid value.
ZERO_GUARD_IDX = (184, 53)
ZERO_GUARD_MIN_NONZERO = 50

# fuel_type from 182: a bad read keeps the last good 182 this long; after
# that fuel_type falls back to the tempCOSet guess, published as unknown.
HEAT_SOURCE_MAX_AGE_SEC = 600
HEAT_SOURCE_PELLETS = 0
HEAT_SOURCE_WOOD = 1

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
# The supervisor is the ONLY writer of 162. The boiler-panel writes 187 and
# 186 and asks for another buffer end through the override topic. While
# fuel_type is unknown (fuel_type_known = 0) the supervisor writes nothing
# except an override.
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
# Exhaust fan (param 194) per fuel: off on pellets (they ran without it before
# the fan was fitted; at night it is only noise), on for wood at once.
EXHAUST_FAN_IDX = 194
EXHAUST_FAN_BY_FUEL = {FUEL_PELLETS: 0, FUEL_WOOD: 1}
WRITE_URL = "http://192.168.8.3/econet/rmNewParam"
WRITE_READBACK_SEC = 30
BUFFER_END_IDX = 162
BUFFER_END_BY_FUEL = {FUEL_PELLETS: 50, FUEL_WOOD: 72}
SUPERVISOR_SETTLE_POLLS = 3        # fuel_type must hold this many polls
SUPERVISOR_RETRY_SEC = 1800        # after a failed write, wait before retrying
CMD_BUFFER_END_TOPIC = "home/boiler/cmd/buffer_end"
CMD_EXHAUST_FAN_TOPIC = "home/boiler/cmd/exhaust_fan"

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
    # Optical flame sensor, % (2026-10-15). The controller declares wood
    # burnt out when it stays below param 25 (5 %) for param 185 (15 min).
    # It looks through the pellet burner opening into the firebox: a fresh
    # wood load covers it (dark, switch to pellets too early), glowing embers
    # keep it bright (no switch for hours).
    "tempOpticalSensor": "flame",
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

# Exhaust fan override, same form on home/boiler/cmd/exhaust_fan:
# {"value": 0, "until": "2026-10-28T07:00"} keeps 194 off on embers at night
# (Janis, 2026-10-27: noise for nothing; the panel's probes then use the
# blower only).
exhaust_fan_override = None

def on_cmd_exhaust_fan(client, userdata, msg):
    global exhaust_fan_override, exhaust_fan_last_fail
    payload = msg.payload.decode("utf-8", "replace").strip()
    if not payload:
        with override_lock:
            exhaust_fan_override = None
        log.info("Exhaust fan override cleared")
        return
    try:
        cmd = json.loads(payload)
        value = int(cmd["value"])
        until = parse_until(str(cmd["until"]))
        if value not in (0, 1):
            raise ValueError("value must be 0 or 1")
    except (ValueError, KeyError, TypeError) as e:
        log.warning("Ignoring %s payload %r: %s", CMD_EXHAUST_FAN_TOPIC, payload[:200], e)
        return
    with override_lock:
        exhaust_fan_override = (value, until)
    exhaust_fan_last_fail = float("-inf")
    log.info("Exhaust fan override %s until %s", value, until.isoformat())

def active_exhaust_fan_override():
    global exhaust_fan_override
    with override_lock:
        ov = exhaust_fan_override
    if ov is None:
        return None
    value, until = ov
    if datetime.now(timezone.utc) < until:
        return value
    with override_lock:
        if exhaust_fan_override == ov:
            exhaust_fan_override = None
    log.info("Exhaust fan override %s expired at %s", value, until.isoformat())
    return None

def on_connect(client, userdata, flags, rc, properties=None):
    # Subscribe here so a broker reconnect re-subscribes; the retained
    # command is delivered again, so an override survives a restart.
    if rc == 0:
        client.subscribe(CMD_BUFFER_END_TOPIC, qos=1)
        client.subscribe(CMD_EXHAUST_FAN_TOPIC, qos=1)

# ---------- MQTT ----------
mqttc = mqtt.Client(client_id="boiler_poller")
mqttc.on_connect = on_connect
mqttc.message_callback_add(CMD_BUFFER_END_TOPIC, on_cmd_buffer_end)
mqttc.message_callback_add(CMD_EXHAUST_FAN_TOPIC, on_cmd_exhaust_fan)
mqttc.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqttc.loop_start()

# Log lines also to MQTT home/log/boiler-poller as "<I|W|E> <message>", not
# retained, for Grafana "Logs" (Loki via Telegraf, 2026-10-15). journalctl
# keeps the full log; lines made while MQTT is down are only there.
LOG_TOPIC = "home/log/boiler-poller"

class MqttLogHandler(logging.Handler):
    LEVEL = {logging.INFO: "I", logging.WARNING: "W", logging.ERROR: "E", logging.CRITICAL: "E"}

    def emit(self, record):
        level = self.LEVEL.get(record.levelno)
        if level is None or not mqttc.is_connected():
            return
        try:
            mqttc.publish(LOG_TOPIC, f"{level} {record.getMessage()}", qos=0, retain=False)
        except Exception:
            self.handleError(record)

log.addHandler(MqttLogHandler())
log.info("boiler_poller started, log also on %s", LOG_TOPIC)

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
        guards = {idx: data[idx]["value"] for idx in ZERO_GUARD_IDX}
        nonzero = sum(1 for p in data if isinstance(p, dict) and p.get("value"))
    except (TypeError, IndexError, KeyError):
        log.warning("Params answer empty or short (%s entries), discarded",
                    len(data) if isinstance(data, list) else "no")
        return None
    if not all(guards.values()) or nonzero < ZERO_GUARD_MIN_NONZERO:
        log.warning("Params answer has %s, %d non-zero values (zero glitch), discarded",
                    guards, nonzero)
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

fuel_unknown_reason = None

def fuel_unknown(reason):
    """Log why fuel_type is unknown, once per change of reason."""
    global fuel_unknown_reason
    if reason != fuel_unknown_reason:
        log.warning("fuel_type unknown: %s", reason)
        fuel_unknown_reason = reason
    return None

heat_source = None           # last good 182
heat_source_at = float("-inf")  # time.monotonic() of that read

def derive_fuel_type(curr, params, params_fresh):
    """Return (fuel_type, known). From param 182 (inverted encoding: 182 = 1
    wood -> FUEL_WOOD 0, 182 = 0 pellets -> FUEL_PELLETS 1), known = True.
    182 = 2 (backup boiler) or any other value: (None, False). A bad params
    read keeps the last good 182 for HEAT_SOURCE_MAX_AGE_SEC; after that the
    tempCOSet guess, with known = False so nothing acts on it."""
    global heat_source, heat_source_at, fuel_unknown_reason
    if params_fresh and params.get("heat_source") is not None:
        heat_source = params["heat_source"]
        heat_source_at = time.monotonic()
    if time.monotonic() - heat_source_at <= HEAT_SOURCE_MAX_AGE_SEC:
        if heat_source in (HEAT_SOURCE_WOOD, HEAT_SOURCE_PELLETS):
            fuel_unknown_reason = None
            return (FUEL_WOOD if heat_source == HEAT_SOURCE_WOOD else FUEL_PELLETS), True
        fuel_unknown(f"heat source 182 = {heat_source}")
        return None, False
    fuel_unknown("no good read of 182 for 10 min, guessing from tempCOSet")
    return guess_fuel_type(curr, params), False

def guess_fuel_type(curr, params):
    """Fallback: active fuel from tempCOSet vs the setpoint each fuel is
    expected to show. Returns FUEL_WOOD / FUEL_PELLETS, or None when a field
    is missing, both expectations match or neither does.

    The controller raises the boiler setpoint to Buffer loading end (162) +
    param 68 when that is higher than the fuel's own setpoint: since
    2026-10-12 14:01 tempCOSet read 77 in pellet mode = 72 + 5 (one data
    point, a hypothesis). Compared with 53/184 alone, 77 matched neither, the
    old code kept the last value (wood), the supervisor kept 162 = 72 and so
    kept the setpoint at 77: a self-locking loop charging the buffer to 72
    on pellets."""
    if not params or curr is None:
        return fuel_unknown("no data this poll")
    active = curr.get("tempCOSet")
    p53 = params.get("temp_co_set_pellets")
    p184 = params.get("temp_co_set_wood")
    p162 = params.get("buffer_end_set")
    p68 = params.get("temp_co_increase")
    if None in (active, p53, p184, p162, p68):
        return fuel_unknown("field missing")
    raised = p162 + p68
    pellets_exp = max(p53, raised)
    wood_exp = max(p184, raised)
    is_pellets = active == pellets_exp
    is_wood = active == wood_exp
    if is_pellets == is_wood:
        return fuel_unknown(f"tempCOSet={active}, expected pellets={pellets_exp} wood={wood_exp}")
    return FUEL_PELLETS if is_pellets else FUEL_WOOD

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

# Loading pump stall: in wood mode the controller sometimes leaves the boiler
# pump off for 6-16 min after a load while the boiler climbs to 80-85 and
# the buffer is cold (13 of 52 loads 10-13 .. 10-27, with and without the
# panel's embers), ending in Supervision. Cause unknown; this only reports
# it: home/boiler/pump_stalled 1/0 and a W log line at the start.
PUMP_STALL_CO = 72          # boiler at or above this
PUMP_STALL_BUFFER_K = 10    # buffer top at least this much colder than the boiler
PUMP_STALL_POLLS = 2        # condition held this many polls (~2 min)
pump_stall_polls = 0
pump_stall_since = None
pump_stall_max_co = 0.0

def watch_loading_pump(curr, params):
    global pump_stall_polls, pump_stall_since, pump_stall_max_co
    co, upper, lower = curr.get("tempCO"), curr.get("tempUpperBuffer"), curr.get("tempLowerBuffer")
    pump, buffer_end = curr.get("pumpCOWorks"), params.get("buffer_end_set") if params else None
    if None in (co, upper, lower, pump, curr.get("mode")):
        return
    stalled = (curr.get("mode") == 3 and co >= PUMP_STALL_CO and not pump
               and upper <= co - PUMP_STALL_BUFFER_K
               and (buffer_end is None or lower < buffer_end))
    if stalled:
        pump_stall_polls += 1
        pump_stall_max_co = max(pump_stall_max_co, co)
        if pump_stall_polls == PUMP_STALL_POLLS:
            pump_stall_since = time.monotonic() - (PUMP_STALL_POLLS - 1) * POLL_INTERVAL_SEC
            log.warning("Loading pump off with the boiler hot: boiler %.1f, buffer %.1f/%.1f, "
                        "Operation, pump_co_works 0 for %d polls", co, upper, lower, PUMP_STALL_POLLS)
            publish("pump_stalled", 1)
        return
    if pump_stall_since is not None:
        log.info("Loading pump stall over after %d min (boiler max %.1f, now %.1f, pump %s, mode %s)",
                 round((time.monotonic() - pump_stall_since) / 60), pump_stall_max_co, co,
                 1 if pump else 0, curr.get("mode"))
        publish("pump_stalled", 0)
    elif pump_stall_polls == 0:
        publish("pump_stalled", 0)
    pump_stall_polls, pump_stall_since, pump_stall_max_co = 0, None, 0.0

exhaust_fan_last_fail = float("-inf")

def supervise_exhaust_fan(fuel_type, params, stable_polls):
    """Param 194 to EXHAUST_FAN_BY_FUEL: wood at once, pellets after the fuel
    has held SUPERVISOR_SETTLE_POLLS. Unknown fuel: nothing."""
    global exhaust_fan_last_fail
    if not SUPERVISOR_ENABLED or not params:
        return
    override = active_exhaust_fan_override()
    if override is None and fuel_type is None:
        return
    target = override if override is not None else EXHAUST_FAN_BY_FUEL[fuel_type]
    current = params.get("exhaust_fan_enabled")
    if current is None or current == target:
        return
    if override is None and fuel_type == FUEL_PELLETS and stable_polls < SUPERVISOR_SETTLE_POLLS:
        return
    if time.monotonic() - exhaust_fan_last_fail < 120:
        return
    log.info("Supervisor: exhaust fan (194) %s -> %s (%s)", current, target,
             "override" if override is not None else "fuel wood" if fuel_type == FUEL_WOOD else "fuel pellets")
    if write_param(EXHAUST_FAN_IDX, target):
        params["exhaust_fan_enabled"] = target
        publish("exhaust_fan_enabled", target)
        exhaust_fan_last_fail = float("-inf")
    else:
        exhaust_fan_last_fail = time.monotonic()

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

# Heat dump (Janis, 2026-10-27): on wood with a full buffer the boiler climbs
# to its setpoint and goes to Supervision (fans stop, the wood smoulders).
# Raising a room setpoint ends the thermostat's -30 K mixer cut (param 87/89)
# and the house takes the heat: T3 21.4 -> 21.6 at 19:38 took the boiler from
# 76.4 to 67.9 in 10 min. The poller raises the ecoSTER day setpoint of T1
# (floor) and T3 (radiators) to room + HEAT_DUMP_RAISE_K (the thermostat asks
# for heat only below set - 0.2) while it holds, then writes the old value
# back. Wood only: on pellets this would burn pellets. Only panels in day
# mode (ecoSterMode 2), since the write sets STER_TEMP_DAY_n. A setpoint a
# person changes meanwhile becomes the new value and is left alone. Writes
# are checked on the next polls (ecoSterSetTempN), as newParam also answers
# OK and drops writes. The old values are kept in HEAT_DUMP_FILE so a
# restart still restores them.
HEAT_DUMP_ZONES = (1, 3)
HEAT_DUMP_URL = "http://192.168.8.3/econet/newParam"
HEAT_DUMP_FILE = "/var/lib/boiler_poller/heat_dump.json"
HEAT_DUMP_ON_K = 5          # start: boiler at or above its setpoint - 5
HEAT_DUMP_BUFFER_K = 3      # and buffer bottom at or above 162 - 3
# Stop when the fire no longer covers the house: the buffer top 2 K below its
# peak in this dump (~1.6 kWh). The 19:38 test ran into the burn-out: buffer
# top 72.8 -> 67.2 in 35 min (~4 kW out with no fire), the room only +0.1,
# and the boiler held 66-68, so a boiler limit alone would have run 3 h.
HEAT_DUMP_BUFFER_DROP_K = 2
HEAT_DUMP_OFF_K = 10        # stop: boiler at or below its setpoint - 10
HEAT_DUMP_MIN_SEC = 600     # no stop on boiler temperature in the first 10 min
HEAT_DUMP_PAUSE_SEC = 900   # no new dump this soon after one ends
HEAT_DUMP_MAX_SEC = 10800   # at most 3 h per dump
HEAT_DUMP_RAISE_K = 0.4
HEAT_DUMP_MAX_K = 1.0       # never more than this above the old setpoint
HEAT_DUMP_WRITES = 5        # tries per value, one per poll
heat_dump_since = None      # monotonic start, None = off
heat_dump_ended = float("-inf")
heat_dump_upper_max = None  # buffer top peak during this dump
heat_dump_zones = {}        # zone -> {base, target, confirmed, restoring, writes}
heat_dump_hands_off = set() # zones a person changed during this dump

def heat_dump_save():
    try:
        os.makedirs(os.path.dirname(HEAT_DUMP_FILE), exist_ok=True)
        tmp = HEAT_DUMP_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({str(n): [z["base"], z["target"]] for n, z in heat_dump_zones.items()}, f)
        os.replace(tmp, HEAT_DUMP_FILE)
    except OSError as e:
        log.warning("Could not save %s: %s", HEAT_DUMP_FILE, e)

def heat_dump_load():
    """Setpoints raised before a restart: restored unless the dump goes on."""
    try:
        with open(HEAT_DUMP_FILE) as f:
            saved = json.load(f)
    except FileNotFoundError:
        return
    except (ValueError, OSError) as e:
        log.warning("%s unreadable: %s", HEAT_DUMP_FILE, e)
        return
    for n, (base, target) in saved.items():
        if target is None:
            continue
        heat_dump_zones[int(n)] = {"base": float(base), "target": float(target), "confirmed": True,
                                   "restoring": True, "writes": 0}
        log.info("Heat dump: T%s to restore to %.1f after a restart", n, float(base))

def heat_dump_write(n, value):
    try:
        resp = requests.get(HEAT_DUMP_URL,
                            params={"newParamName": f"STER_TEMP_DAY_{n}", "newParamValue": f"{value:.1f}"},
                            auth=ECONET_AUTH, timeout=HTTP_TIMEOUT_SEC)
        resp.raise_for_status()
        if resp.json().get("result") != "OK":
            log.warning("Write STER_TEMP_DAY_%d=%.1f refused: %s", n, value, resp.text[:200])
    except (requests.RequestException, ValueError) as e:
        log.warning("Write STER_TEMP_DAY_%d=%.1f failed: %s", n, value, e)

def heat_dump_zone(n, curr, on):
    """One poll for one panel: raise, follow the room up, or restore."""
    set_t, room = curr.get(f"ecoSterSetTemp{n}"), curr.get(f"ecoSterTemp{n}")
    if set_t is None:
        return
    set_t = round(set_t, 1)
    z = heat_dump_zones.get(n)
    if z is None:
        if not on or room is None or curr.get(f"ecoSterMode{n}") != 2 or n in heat_dump_hands_off:
            return
        z = {"base": set_t, "target": None, "confirmed": False, "restoring": False, "writes": 0}
    want = z["base"] if not on else min(z["base"] + HEAT_DUMP_MAX_K, round(room + HEAT_DUMP_RAISE_K, 1))
    if z["confirmed"] and z["target"] is not None and abs(set_t - z["target"]) >= 0.05 \
            and abs(set_t - z["base"]) >= 0.05:
        log.info("Heat dump: T%d set to %.1f by a person, left alone", n, set_t)
        heat_dump_hands_off.add(n)
        heat_dump_zones.pop(n, None)
        heat_dump_save()
        return
    if not on:
        if abs(set_t - z["base"]) < 0.05:
            if n in heat_dump_zones:
                log.info("Heat dump: T%d back to %.1f (room %s)", n, z["base"],
                         f"{room:.1f}" if room is not None else "?")
                heat_dump_zones.pop(n)
                heat_dump_save()
            return
        if not z["restoring"]:
            z.update(restoring=True, writes=0)
    else:
        if z["restoring"]:          # on again before the old value was back
            z.update(restoring=False, writes=0)
        if z["target"] is not None and abs(set_t - z["target"]) < 0.05:
            if not z["confirmed"]:
                log.info("Heat dump: T%d = %.1f confirmed (room %.1f, was %.1f)", n, set_t, room, z["base"])
            z["confirmed"] = True
            if want <= z["target"] + 0.05:
                return              # the room follows; raise again as it warms
        elif z["target"] is None and want <= set_t + 0.05:
            return                  # room already well below the setpoint
        if z["target"] is None or want > z["target"] + 0.05:
            z.update(target=want, confirmed=False, restoring=False, writes=0)
            heat_dump_zones[n] = z
            heat_dump_save()
    value = z["base"] if z["restoring"] else z["target"]
    if z["writes"] >= HEAT_DUMP_WRITES:
        if z["writes"] == HEAT_DUMP_WRITES:
            log.warning("Heat dump: T%d still %.1f after %d writes of %.1f, giving up", n, set_t,
                        HEAT_DUMP_WRITES, value)
            z["writes"] += 1
        return
    z["writes"] += 1
    if z["writes"] == 1:
        log.info("Heat dump: T%d %.1f -> %.1f (room %s)", n, set_t, value,
                 f"{room:.1f}" if room is not None else "?")
    heat_dump_write(n, value)

def heat_dump(curr, params, fuel_type):
    global heat_dump_since, heat_dump_ended, heat_dump_upper_max
    co, co_set, lower = curr.get("tempCO"), curr.get("tempCOSet"), curr.get("tempLowerBuffer")
    upper = curr.get("tempUpperBuffer")
    buffer_end, mode = params.get("buffer_end_set") if params else None, curr.get("mode")
    now = time.monotonic()
    if heat_dump_since is None:
        if (fuel_type == FUEL_WOOD and mode == 3 and None not in (co, co_set, lower, upper, buffer_end)
                and now - heat_dump_ended >= HEAT_DUMP_PAUSE_SEC
                and co >= co_set - HEAT_DUMP_ON_K and lower >= buffer_end - HEAT_DUMP_BUFFER_K):
            heat_dump_since, heat_dump_upper_max = now, upper
            log.info("Heat dump on: boiler %.1f/%s, buffer %.1f/%.1f (162 = %s)", co, co_set, upper, lower,
                     buffer_end)
    else:
        if upper is not None:
            heat_dump_upper_max = max(heat_dump_upper_max, upper)
        reason = ("pellets" if fuel_type == FUEL_PELLETS
                  else f"mode {mode}" if mode not in (3, 4)
                  else "3 h" if now - heat_dump_since >= HEAT_DUMP_MAX_SEC
                  else f"buffer top {upper:.1f}, peak {heat_dump_upper_max:.1f}"
                  if upper is not None and upper <= heat_dump_upper_max - HEAT_DUMP_BUFFER_DROP_K
                  else f"boiler {co:.1f}" if (co is not None and co_set is not None and co <= co_set - HEAT_DUMP_OFF_K
                                               and now - heat_dump_since >= HEAT_DUMP_MIN_SEC)
                  else None)
        if reason:
            log.info("Heat dump off after %d min (%s)", round((now - heat_dump_since) / 60), reason)
            heat_dump_since, heat_dump_ended = None, now
            heat_dump_hands_off.clear()
    on = heat_dump_since is not None
    publish("heat_dump", 1 if on else 0)
    for n in HEAT_DUMP_ZONES:
        heat_dump_zone(n, curr, on)

heat_dump_load()

while running:
    # Service parameters: not critical for heater control, so a failure here
    # only logs and leaves `online` alone.
    params_fresh = False
    if poll_count % PARAMS_EVERY_N_POLLS == 0:
        fetched = fetch_params()
        if fetched:
            params = fetched
            params_fresh = True
            for topic, val in params.items():
                publish(topic, val)
    poll_count += 1

    curr = fetch_once()

    publish_counters(params)

    # fuel_type is published whenever there is a value (182, or the fallback
    # guess); only a value from 182 counts as known. Unknown resets the
    # stability count, so the supervisor and the panel's 186/187 guard only
    # act on 182. 182 = 2 (backup boiler): no fuel_type at all.
    fuel_type, fuel_known = derive_fuel_type(curr, params, params_fresh)
    if fuel_type is not None and fuel_type != last_fuel_type:
        log.info("fuel_type -> %s (%s)", "pellets" if fuel_type == FUEL_PELLETS else "wood",
                 "param 182" if fuel_known else "tempCOSet guess")
        last_fuel_type = fuel_type
        fuel_stable_polls = 0
    elif fuel_known:
        fuel_stable_polls += 1
    if not fuel_known:
        fuel_stable_polls = 0
    if fuel_type is not None:
        publish("fuel_type", fuel_type)
    publish("fuel_type_known", 1 if fuel_known else 0)
    supervise_buffer_end(fuel_type if fuel_known else None, params, fuel_stable_polls)
    supervise_exhaust_fan(fuel_type if fuel_known else None, params, fuel_stable_polls)
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
        watch_loading_pump(curr, params)
        heat_dump(curr, params, fuel_type if fuel_known else None)

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
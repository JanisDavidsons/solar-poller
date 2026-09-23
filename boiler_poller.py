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
import signal
import sys
import time

import paho.mqtt.client as mqtt
import requests

# ---------- Config ----------
# Plum ecoNET 300 module on the LAN. Same data the cloud sees, but no
# auth dance: just HTTP Basic with the module's web UI credentials.
ECONET_URL  = "http://192.168.8.3/econet/regParams"
ECONET_AUTH = ("admin", "admin")

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
    "fuelLevel":       "fuel_level",
    "fuelStream":      "fuel_stream",
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
    payload = f"{value:.2f}" if isinstance(value, float) else str(value)
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

log.info("Polling %s every %ds, publishing to %s://%s:%d under %s/*",
         ECONET_URL, POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_PREFIX)

consecutive_failures = 0

while running:
    curr = fetch_once()
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
publish("online", 0)
mqttc.loop_stop()
mqttc.disconnect()
sys.exit(0)
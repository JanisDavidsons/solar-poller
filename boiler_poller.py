#!/usr/bin/env python3
"""
boiler_poller.py

Polls econet24.com for boiler/buffer/CWU temperatures every 60 seconds and
publishes them to MQTT. Intended to run as a systemd service alongside
solar_poller.

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
ECONET_URL    = "https://econet24.com/service/getDeviceParams"
ECONET_UID    = "3G49NB0P32D9K0SB00500"
ECONET_COOKIE = (
    'csrftoken=ch27diYjpY44zjXhLC1OY5M5qk1EHCbB; '
    '_mlmlc="b798b0ed-6c3c-4d75-ac0f-37d0cd85363a?'
    '2e393dd6-3964-4eff-abf7-4fb8889fe723?B?69F323EC%'
    '9085790999d2bf11e040714948a09a0f167184e3f4d84ada08fbf40b5d5f0230?'
    'w:NPL0WH9f7yQA3aji_zHUT39E3B3Qep8ASKT1FC5ERA0"'
)
HEADERS = {
    "X-Requested-With": "XMLHttpRequest",
    "Cookie": ECONET_COOKIE,
}

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
    "mode":            "mode",
}

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
            params={"uid": ECONET_UID},
            headers=HEADERS,
            timeout=HTTP_TIMEOUT_SEC,
        )
        resp.raise_for_status()
        data = resp.json()
        return data.get("curr", {})
    except requests.RequestException as e:
        log.warning("HTTP fetch failed: %s", e)
        return None
    except (ValueError, KeyError) as e:
        log.warning("JSON parse failed: %s", e)
        return None

log.info("Polling econet24 every %ds, publishing to %s://%s:%d under %s/*",
         POLL_INTERVAL_SEC, "mqtt", MQTT_HOST, MQTT_PORT, MQTT_PREFIX)

consecutive_failures = 0

while running:
    curr = fetch_once()
    if curr is None:
        consecutive_failures += 1
        publish("online", 0)
        # Backoff: keep trying every minute, but log loudly after sustained failure
        if consecutive_failures == 5:
            log.error("5 consecutive fetch failures — cookies may have expired")
    else:
        if consecutive_failures > 0:
            log.info("Recovered after %d failure(s)", consecutive_failures)
        consecutive_failures = 0
        publish("online", 1)

        for field, sub_topic in FIELD_MAP.items():
            val = curr.get(field)
            if val is None:
                continue
            publish(sub_topic, val)

        # One-line summary log so journalctl shows a heartbeat
        hot_water = curr.get("tempCWU")
        upper = curr.get("tempUpperBuffer")
        lower = curr.get("tempLowerBuffer")
        log.info("hot_water=%.1f°C upper=%.1f°C lower=%.1f°C",
                 hot_water if hot_water else 0,
                 upper if upper else 0,
                 lower if lower else 0)

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
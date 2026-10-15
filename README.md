# solar-poller

The two pollers on the OrangePi One: `solar_poller` reads the SDM630 meter and
switches the four heater elements on solar surplus; `boiler_poller` reads the
ecoNET 300 and publishes the boiler. Both publish to the Mosquitto broker on
the Pi.

## What runs where

| host | what |
|---|---|
| OrangePi One `192.168.8.50`, ssh alias `pi`, user `janis` | Mosquitto `:1883` (anonymous, `mosquitto/network.conf`), `solar-poller.service`, `boiler-poller.service` (units in `systemd/`) |
| ecoNET 300 `192.168.8.3` | HTTP basic auth `admin:admin`: `/econet/regParams`, `/econet/rmParamsData`, writes `/econet/rmNewParam?newParamIndex=N&newParamValue=V` |
| winsrv `192.168.8.20` | Telegraf, InfluxDB `:8086`, Grafana `:3000`: repo `home-energy-monitor` |
| boiler-panel at the boiler | Mega + NodeMCU: repo `boiler-panel` |

- `solar_poller`: `/usr/local/bin/solar_poller`, built on the Pi from
  `solar_poller.cpp`. SDM630 on `/dev/sdm630` (`udev/99-sdm630.rules`),
  Modbus RTU 9600 8N1, slave 1, every 5 s. Heater SSRs on `/dev/gpiochip0`.
- `boiler_poller`: `python3 /home/janis/boiler_poller.py`, every 60 s.
  Pellet counter persisted in `/var/lib/boiler_poller/counters.json`.

## Deploy

```sh
./deploy.sh
```

Copies both sources to `pi:/home/janis/`, builds `solar_poller` on the Pi
(`-lmodbus -lmosquitto -lgpiod`), restarts both services and prints their
last log lines. It always rebuilds and restarts `solar-poller` as well.

Logs: `ssh pi "journalctl -u boiler-poller -n 50 --no-pager"` (same for
`solar-poller`). `boiler_poller` also publishes its log lines on MQTT
`home/log/boiler-poller` (`<I|W|E> <message>`).

## Data

MQTT, broker `192.168.8.50:1883`:

- `home/solar/...`: per-phase power, `power_total`, `heater_<el>_state` (0/1),
  `heater_<el>_threshold_on/_off`; every 5 s, not retained.
- `home/boiler/<metric>`: regParams fields and the params in `PARAM_MAP`
  (list in `boiler_poller.py`); every 60 s, retained, bare numbers.
- `home/boiler/cmd/buffer_end`: override for param 162, retained JSON
  `{"value": 66, "until": "2026-10-12T22:00:00+03:00"}`; empty message clears.
- History: InfluxDB on winsrv (`home` 90 d raw, `home_long` 1-min means and
  daily totals), token `INFLUX_TOKEN` in `home-energy-monitor/.env`; Grafana
  http://192.168.8.20:3000, dashboard 5 Boiler, 7 Logs.
- ecoMAX parameter list: `docs/ecomax-params-2026-10-14.md`.

## Rules

- The `boiler_poller` supervisor is the only writer of param 162. A manual
  write is reverted within minutes; use the override topic.
- boiler-panel writes 186 (blower) and 187 (exhaust fan), nothing else.
- The monitoring stack only subscribes; nothing there publishes to `home/#`.
- Do not release a heater GPIO line to switch it off: gpiochip0 keeps the
  last driven value. Off = drive the line inactive.
- No element runs without a fresh meter reading (6 failed reads: all off)
  or a fresh tank temperature (none for 300 s: that element off).

## boiler_poller settings

- `fuel_type` (0 wood, 1 pellets) from param 182 "Siltuma avots" (0 pellet
  burner, 1 wood grate, 2 backup: no fuel_type). `fuel_type_known` = 1 only
  when it comes from 182. A bad params read keeps the last 182 for 10 min,
  then the tempCOSet fallback counts as unknown.
- Supervisor, param 162 (buffer loading end): pellets 50, wood 72, written
  after 3 stable polls; nothing on unknown fuel; 30 min wait after a failed
  write.
- Zero guard on rmParamsData: 184 and 53 must be non-zero and the answer
  must have at least 50 non-zero values, else the read is discarded.
- Pellet kg = feeder on-time x param 48 x `PELLET_CALIBRATION` (1.0).

## solar_poller settings

| element | GPIO line | phase | load | probe | cap / resume |
|---|---|---|---|---|---|
| L1 DHW | 9 | L1 | 1245 W | `temp_hot_water` | 70 / 65 |
| L1b buffer middle | 7 | L1 | 1240 W | `temp_upper_buf` | 88 / 85 |
| L2 buffer bottom | 10 | L2 | 1560 W | `temp_lower_buf` | 75 / 70, off if upper > 88 |
| L3 buffer top | 8 | L3 | 1245 W | `temp_upper_buf` | 85 / 80 |

- L1b runs only while L1 (same phase) is off at its cap.
- Tariffs (c/kWh): grid peak 18.09, cheap 11.60 (02:00-06:00 and
  12:00-16:00 Europe/Riga), pellets 6.0, export 1.03. Solar share needed:
  ~71 % peak, ~53 % cheap.
- Thresholds per element from its load and the tariff, band 300 W
  (`TURN_ON_HEADROOM_W` 150 + `TURN_OFF_HYSTERESIS_W` 150); on after 30 s,
  off after 60 s. Values in force are published as `heater_<el>_threshold_*`.

## History

`git log` and Grafana "7. Logs".

# home-energy-monitor

A read-only monitoring server for the existing solar and boiler system. It
collects everything published on the home MQTT broker into InfluxDB and serves
Grafana dashboards. It observes and nothing else.

InfluxDB 2 + Telegraf + Grafana, one docker-compose stack, meant to run on a
spare Windows desktop under Docker Desktop with the WSL2 backend.

## Read-only guarantee

The only MQTT plugin in this stack is `inputs.mqtt_consumer`, which subscribes.
Nothing here publishes to `home/#` and nothing writes toward the OrangePi.
`solar_poller` and `boiler_poller` stay exactly as they are.

If you extend the stack, do not add `outputs.mqtt`, and do not add a Grafana
plugin or alert contact point that can write back to the broker. The producers
own `home/#`.

## Windows prep checklist

Do this once on the machine that will run the stack.

1. **Install Docker Desktop with the WSL2 backend.** In Docker Desktop:
   Settings, General, tick *Use the WSL 2 based engine*.
2. **Enable autostart.** Settings, General, tick *Start Docker Desktop when you
   sign in*. Then set Windows to sign the account in automatically, or the
   stack will not come back after a reboot until someone logs in.
3. **Disable sleep and hibernate.** In an elevated PowerShell:

   ```powershell
   powercfg /change standby-timeout-ac 0
   powercfg /change hibernate-timeout-ac 0
   powercfg /change monitor-timeout-ac 20
   powercfg /hibernate off
   ```

   Also check Settings, System, Power, that *Sleep* is set to *Never* on mains.
4. **Let the box take updates without losing the stack.** Every container in
   `docker-compose.yml` is `restart: unless-stopped`, so they come back after a
   reboot or a Docker Desktop update by themselves.
5. **Give WSL2 enough memory.** Create `%UserProfile%\.wslconfig`:

   ```ini
   [wsl2]
   memory=4GB
   processors=2
   ```

   InfluxDB is the hungry one. 4 GB is comfortable for this data volume.
6. **Open the LAN firewall for Grafana** if you want to reach it from other
   machines, elevated PowerShell:

   ```powershell
   New-NetFirewallRule -DisplayName "Grafana LAN" -Direction Inbound `
     -LocalPort 3000 -Protocol TCP -Action Allow -Profile Private
   ```

### Where the data actually lives

The compose file uses **named volumes**, not `C:\` bind mounts, so the stack
stays portable and survives Docker Desktop updates. With the WSL2 backend the
volumes live inside the Docker Desktop WSL distro, reachable from Explorer at
roughly:

```
\\wsl$\docker-desktop-data\data\docker\volumes\          (older Docker Desktop)
\\wsl.localhost\docker-desktop\mnt\docker-desktop-disk\data\docker\volumes\   (newer)
```

Treat those paths as read-only curiosities. Do not edit anything in there.
Back up through Docker instead, which works regardless of layout:

```powershell
# InfluxDB data to .\backup\influxdb-data.tar.gz
docker run --rm -v home-energy-monitor_influxdb-data:/data -v ${PWD}\backup:/backup `
  busybox tar czf /backup/influxdb-data.tar.gz -C /data .
```

A Docker Desktop **factory reset** or *Clean / Purge data* deletes named
volumes. An ordinary update does not. Take a backup before either.

## First run

```powershell
Copy-Item env.example .env
notepad .env          # set the passwords and INFLUX_TOKEN, check MQTT_HOST
docker compose up -d
docker compose logs -f telegraf
```

Within a minute or so the Telegraf log should show it connected and stop
complaining. Then:

1. Open Grafana at <http://localhost:3000>. Log in as `admin` with the
   password from `.env`. The datasource and all six dashboards provision
   themselves; LAN viewers can read them without logging in.
2. Confirm the InfluxDB tasks were applied:

   ```powershell
   docker compose logs influx-tasks
   ```

   You want `downsample_1m` and `daily_aggregates` in the list at the end.
3. Pin the topic-semantics annotation, once:

   ```powershell
   $env:GRAFANA_ADMIN_PASSWORD = "<the value from .env>"
   powershell -ExecutionPolicy Bypass -File .\scripts\seed-annotations.ps1
   ```
4. Open dashboard **5. Boiler** and look at *Every boiler field seen in this
   range*. That table is the real list of what `boiler_poller` publishes. See
   "Boiler field names" below.

## Data model

Telegraf ingests all of `home/#` generically, so a new topic on the Pi shows up
without a config change. Topics map to InfluxDB like this:

| | |
|---|---|
| measurement | `solar` or `boiler`, the second topic segment |
| field | `value`, a float parsed from the bare numeric payload |
| tag `topic` | the full MQTT topic, kept for discovery |
| tag `metric` | the leaf segment, e.g. `power_L1`, `heater_l1_state`, `tempCWU` |
| tag `quantity` | `power` or `heater_state`, solar only |
| tag `phase` | `L1`, `L2`, `L3`, `total` |
| tag `element` | `l1`, `l1b`, `l2`, `l3`, heaters only |
| tag `label` | human name of the heater element |

Payloads that are not numeric are dropped and logged as a parse error. If a
producer ever starts publishing JSON on a topic, there is a commented second
consumer at the bottom of `telegraf/telegraf.conf` to copy.

### Buckets

| Bucket | Retention | Contents |
|---|---|---|
| `home` | 90 days | raw, every sample as published |
| `home_long` | infinite | 1-minute means, plus the daily rollups |

`home_long` is the multi-year history and the reason the stack exists. Two
InfluxDB tasks keep it filled, both applied on every stack start by the
`influx-tasks` service:

- **`downsample_1m`** runs every 5 minutes, writing 1-minute means of the last
  15 minutes. The overlap is idempotent and covers a late flush from Telegraf.
- **`daily_aggregates`** runs daily, writing per-day totals for the last two
  local days, so a missed run heals itself on the next one:

  | measurement | tags | fields |
  |---|---|---|
  | `daily_grid` | `phase`, `direction` (import/export) | `kwh` |
  | `daily_heater` | `element`, `label` | `kwh`, `hours_on` |
  | `daily_boiler` | `metric` | `total_per_day` |

Raw data expires after 90 days. If `downsample_1m` stops running, the history
behind it is gone for good, so treat a failing `influx-tasks` service as an
outage rather than a cosmetic problem. Check it with:

```powershell
docker compose run --rm influx-tasks     # re-applies, prints the task list
```

Both task definitions are plain Flux under `influxdb/tasks/`, with
`__RAW_BUCKET__`, `__LONGTERM_BUCKET__` and `__ORG__` substituted at apply
time. Edit the file, re-run the service, done.

## Dashboards

All six provision from `grafana/dashboards/*.json` into the *Solar monitor*
folder. Times are Europe/Riga throughout.

1. **Live power** — three phase series with a zero line, the heater timeline
   aligned underneath, then a panel per phase carrying that phase's four
   switching thresholds as dashed lines.
2. **Heater activity** — state timeline of the four elements, duty cycle, and
   on-time and energy over the selected range.
3. **Temperatures** — DHW and buffer temperatures against their 70 °C and
   85 °C caps, flue gas on its own panel.
4. **Energy** — daily kWh diverted per heater, and daily import against export
   per phase, computed from the raw bucket over a short range.
5. **Boiler** — mode and fuel timeline with pump lanes and the two ecoSTER
   room-panel visibility lanes, temperatures, combustion, panel visibility
   as a percentage, plus a field explorer. See "Room panel visibility".
6. **Long-term history** — the same quantities read from the daily rollups,
   including month-by-month year-over-year bars. Use this for anything beyond
   a couple of weeks; it reads a few thousand points where dashboard 4 would
   read millions.

### Room panel visibility

Two wireless eSTER_x40 room panels (ISM_xSMART radio modules on the ecoMAX
RS485 socket) feed the floor (Mixer 1) and radiator (Mixer 3) circuits.
`boiler_poller` publishes `room_temp1`, `room_set1`, `room_temp3`,
`room_set3`, `temp_mix1_set`, `temp_mix3_set` from `regParams`. When the
controller has no reading from a panel, `room_tempN` is simply absent.

Dashboard 5 derives "Panel N visible" from that: a window counts as *ok* when
`room_tempN` was present alongside `temp_mixN_set`, *lost* otherwise. Two
views:

- lanes in the *Boiler mode* timeline (binary; at long ranges a window is *ok*
  if any poll in it saw the panel, so short dropouts vanish there);
- *Panel visibility (%)* below *Combustion*: share of polls per window in
  which the panel was seen. Use this for trends.

Why it matters (confirmed 2026-10-05 over 3 days, 4168 polls): while a panel
is seen, its mixer setpoint is the thermostat-lowered value (Mixer 3: 25 °C)
and the mixer pump is off. In every poll where the panel was missing the
controller fell back to the full setpoint (55 °C) and in 17 % of them ran the
pump. All 1322 setpoint-55 polls and 342 of 344 MIX3 pump-on polls fell inside
Panel 3 dropouts. With an 80 °C buffer each pump pulse pushes hot water into
radiators in rooms that are already warm. That is the "mixer-pump pulsing"
noted earlier, and it is a radio-coverage problem, not a controller fault.

Baseline before any fix (2026-10-02..05): Panel 1 seen ~91 %, Panel 3 seen
43–66 % per day. Dropouts are mostly 30–90 s (one or two missed one-minute
radio packets), occasionally 10–15 min. The two panels dropped independently,
never together, which rules out the RS485 side.

### Tariff windows

Dashboards 1 and 2 shade 02:00–06:00 and 12:00–16:00 Europe/Riga as Grafana
time regions. The reading baked into the panel descriptions is that those are
the cheap-tariff windows, so the cheap threshold set applies inside them and
the peak set outside. If that is backwards, the fix is in each dashboard's
annotation list and in the panel descriptions, nowhere else.

### Adding a panel

1. Add it in the Grafana UI and get it looking right. Provisioned dashboards
   are editable (`allowUiUpdates: true`), so this works.
2. Dashboard settings, **JSON Model**, copy the whole thing.
3. Paste it over the matching file in `grafana/dashboards/`, keeping the `uid`
   and bumping `version`.
4. `docker compose restart grafana`, or just wait 30 seconds for the provider
   to pick it up.

Edits made only in the UI survive until the file changes, then the file wins.
Step 3 is what makes a change permanent.

Queries are Flux. `v.defaultBucket` resolves to the raw bucket from the
datasource; dashboard 6 uses its own hidden `longterm_bucket` constant
variable, which must match `INFLUX_LONGTERM_BUCKET` in `.env` if you rename
the bucket.

## Topic semantics changed on 2026-09-21

The heater topic mapping was corrected on 2026-09-21. From that date:

- `heater_l1_state` — DHW, phase L1, 1245 W
- `heater_l1b_state` — buffer middle, L1, 1240 W
- `heater_l2_state` — buffer bottom, L2, 1560 W
- `heater_l3_state` — buffer top, L3, 1245 W

Before that date `l1b` and `l3` labelled the opposite buffer elements.
**Comparing those two topics across that line by name gives you nonsense.**

This stack started collecting on or after 2026-09-21, so everything it holds
uses the corrected mapping. The annotation seeded by
`scripts/seed-annotations.ps1` marks the boundary on every dashboard, which
matters if history from before that date is ever imported into `home_long`.

The human labels live in one place, the `processors.enum` block in
`telegraf/telegraf.conf`. The `element` tag always keeps the raw topic
identity, so relabelling never rewrites what was recorded.

## Boiler field names

`boiler_poller` republishes the ecoNET 300 `regParams`, and the exact field
names are whatever that produces. Nothing here hardcodes a list: ingest is
generic, and the panels that need specific fields match on a case-insensitive
regex against the `metric` tag.

Once you can see the real names in the explorer table on dashboard 5, tighten:

- `grafana/dashboards/03-temperatures.json` — the flue-gas regex and the four
  named temperature fields
- `grafana/dashboards/05-boiler.json` — the output, fan/flame, fuel stream and
  fuel level regexes
- `influxdb/tasks/daily_aggregates.flux` — the regex in the boiler section,
  then `docker compose run --rm influx-tasks`

## Constraints this stack keeps

- Subscribe-only MQTT. No publishing, no writes toward the Pi.
- No changes required to `solar_poller` or `boiler_poller`.
- No Windows paths inside `docker-compose.yml`. Named volumes only; the one
  `C:\` path in this README is the backup command, as allowed.
- Europe/Riga for dashboards and tariff windows.

## Not built, on purpose

Home Assistant, alerting and notifications, control of any kind, heat-pump
integration. Grafana alerting is available in the image and the data model
suits it, but nothing is configured. Adding alerts later means adding a
contact point and rules in Grafana; it does not mean touching the ingest path.

## Troubleshooting

**Telegraf logs parse errors.** A topic under `home/#` is publishing something
that is not a number. Find it with the `topic` tag in InfluxDB, then either
exclude it from `topics` or add a JSON consumer.

**Dashboards are empty.** Check `docker compose logs telegraf` for a broker
connection, then query InfluxDB directly at <http://localhost:8086> with the
admin credentials from `.env`.

**`influx-tasks` keeps restarting.** It retries five times, then stops. Read
`docker compose logs influx-tasks`: usually a wrong `INFLUX_TOKEN` or an org
name that does not match. InfluxDB itself is unaffected, and raw ingest carries
on, but the long-term history is not being written until this is fixed.

**Data stopped after a network blip, Pi still publishing.** Seen 2026-10-05:
the switch restarted at 15:44, the Pi and both pollers carried on, but Telegraf
inside WSL2 kept a dead MQTT session and wrote nothing for 90 minutes.
`docker compose restart telegraf` fixed it. Two defences are in place:

- `telegraf.conf` sets `keepalive = "30s"` and `ping_timeout = "5s"`, so a
  dead session is noticed within about 35 s and reconnected.
- `scripts/telegraf-watchdog.ps1` checks the age of the newest `boiler` point
  in InfluxDB. Older than 3 min: `docker compose restart telegraf`; still
  stale on the next run: `wsl --shutdown` and `docker compose up -d`. Register
  it as a 5-minute scheduled task (command in the script header). It logs to
  `scripts/watchdog.log`. The task runs `scripts/telegraf-watchdog-hidden.vbs`,
  a wscript wrapper, so no console window flashes every 5 minutes (a direct
  `powershell -File` call does, even with `-WindowStyle Hidden`).

Messages published while Telegraf was down are gone: the consumer uses QoS 0
and no persistent session, by choice. Recovery, not replay, is the goal.

**Grafana forgot a dashboard edit.** The file in `grafana/dashboards/` won.
See "Adding a panel".

## Operations log

Dated changes to the stack and the things around it that the dashboards
depend on. Newest first.

### 2026-10-15

- **fuel_type from param 182 "Siltuma avots" (deployed 09:32).** 182 is
  the active heat source: 0 pellet burner, 1 wood grate, 2 backup boiler
  (1 during the 10-14 wood burn, 0 after the automatic switch to pellets).
  fuel_type keeps its meaning (0 wood, 1 pellets, inverted against 182) and
  is `fuel_type_known` = 1 only when it comes from 182; 182 = 2 publishes no
  fuel_type. A bad params read keeps the last good 182 for 10 min, then the
  tempCOSet guess from 10-13 takes over as unknown (supervisor and panel do
  nothing on it). The zero guard is stricter, since 182 = 0 is valid: 184
  and 53 must be non-zero and the answer must have at least 50 non-zero
  values. New topics `heat_source` (182 raw) and `flame` (regParams
  `tempOpticalSensor`, %: the controller calls wood burnt out when it stays
  below param 25 = 5 % for param 185 = 15 min). Also `blower_level` (param
  186, 10-14) for the boiler panel. Checked after deploy: fuel_type 1 =
  heat_source 0, known 1, no supervisor writes. Full parameter inventory:
  `docs/ecomax-params-2026-10-14.md`.

### 2026-10-13

- **fuel_type self-lock fixed in boiler_poller (deployed with the 10-15
  change, where it became the fallback behind param 182).** Since
  2026-10-12 14:01 `temp_co_set` read 77 in pellet/Halted mode. That fits
  max(p53, p162 + p68) with 162 = 72 and param 68 ("Katla temp.
  palielinājums no SKŪ un maisītāja") = 5; one data point, a hypothesis.
  77 matched neither stored setpoint (75 / 80), the old code kept the last
  value (wood), so the supervisor held 162 = 72 in pellet mode, which in turn
  held the setpoint at 77: pellets charged the buffer to 72. Now: pellets
  expected = max(p53, p162 + p68), wood expected = max(p184, p162 + p68);
  exactly one must match, otherwise fuel is unknown. On unknown the
  published `fuel_type` keeps its last value, the new `fuel_type_known`
  is 0, the stability count restarts and the supervisor writes nothing
  (an override still applies). Param 68 is read every poll and published
  as `temp_co_increase`. In 16 days of data the DHW raise never took the
  pellet setpoint from 75 to 80. If the hypothesis is wrong (e.g. pellets
  at 75 with 162 = 72), fuel stays unknown and 162 stays where it is:
  watch `journalctl -u boiler-poller | grep "fuel_type unknown"` after
  deploy.

### 2026-10-09

- **Exhaust fan tests.** Atmos UCJ4C82 induced-draft fan on the flue, wood-mode
  level param 187. With the blower stopped via the DZT loop the fire dies down
  at 20 % and at 30 % alike (flue gas 88 -> 68 and 80 -> 68 in 10 min), so the
  fan cannot feed the fire through a stopped blower; at 100 % the underpressure
  pulls the boiler door shut. 187 left at 30 (20 barely turns the rotor).
  regParams never shows the door state (fanWorks/fanPower unchanged), and no
  ecoNET endpoint exposes it. Panel + relay design in
  `docs/exhaust-fan-controller.md`; the Arduino Mega + TFT from the diesel
  heater project survived a 12 V mishap and is the panel hardware.
- **Wood setpoint 184: 72 -> 80.** Boiler went to Supervision at 72 with the
  buffer bottom at 50 and the loading pump running: the limit was the boiler
  setpoint, not the buffer. Pellet setpoint (53) is 75, so the two no longer
  coincide and `fuel_type` can tell them apart.
- **Return-temperature check.** LK valve set to 55, manual bypass valve ~45
  degrees closed. With boiler 77 / buffer bottom 51 the return at the boiler
  inlet read 68 (analogue, in-pipe) / 64 (digital, on the pipe surface; keep
  the ~4 K offset in mind). So the LK is saturated open and the manual bypass
  does the work, with 13 K headroom to 55. Closing it one step more pulled the
  boiler from 77 to 74 with the fan still on and doubled the buffer-bottom rise
  (~3.5 -> ~6 K/h). Next step to test at the start of a load with a cold
  buffer bottom; keep the analogue reading >= 58.
- **Watchdog task now runs via `scripts/telegraf-watchdog-hidden.vbs`** so no
  console window flashes every 5 minutes.

### 2026-10-12

- **Full-load wood test, 8.4 kg at 12:36** (largest load so far; the chamber
  takes ~6 kg comfortably). Flue peaked at 142 °C (exhaust fan 40 %), boiler
  hit 80 in 30 min and cycled Supervision twice (4 min, then 30+ min) because
  the fire outran the ~8-12 kW transfer. Buffer went from avg 47 to 71 in 3 h
  (~19.5 kWh) plus DHW +12 K and boiler water: about 2.8 kWh/kg with embers
  still burning, against 2.1-2.6 on 2026-10-01 (setpoint now 80, exhaust fan).
  Final at 16:07: buffer uniform 72-74 top to bottom (first full buffer on
  wood), ~21 kWh in the buffer + ~2.5 DHW + ~1.4 boiler water = ~25 kWh,
  3.0 kWh/kg. In wood mode the loading pump kept running with the lower sensor
  above the loading-end value (62, later 72); the "end" threshold does not
  stop the pump while the boiler is hot.
- **boiler_poller supervisor owns param 162** (found 2026-10-12): it writes 62
  for wood / 50 for pellets after 3 stable cycles and reverted a manual 72 at
  15:11. Changed the same day: wood 72, zero-read guard, `exhaust_level`
  from 187, `buffer_end_target`, `buffer_end_overwritten`, and a retained
  override topic `home/boiler/cmd/buffer_end` (`{"value","until"}`) that the
  supervisor itself applies. The panel will not write 162.
- **Lower buffer sensor moved** from its pocket ~10 cm below the boiler
  return port (P1) to P1 level. Below P1 is dead volume; the old position
  lagged the loop by 10-20 min and explains "the buffer never gets full".
  Port map, bottom to top: P1 boiler return (via pump), P2 house returns
  (+ DHW coil return since the DHW rebuild), P3 boiler supply, P4 house
  supply. 20 cm between the ports of each pair.
- **Analogue dials on the tank lag** by 10-15 min; do not compare them with
  pipe readings taken at the same moment.
- **Open:** the `buffer_end_set` line on dashboard 5 showed 62 while param 162
  was 72, so the poller publishes a constant per fuel, not the parameter; fix
  in solar-poller (read 162 from rmParamsData). Possible gravity circulation
  through the DHW coil back into P2 (bottom rise rate doubled with the DHW
  valves closed); confirm with a thermometer on the coil return with the
  DHW pump off, then a check valve. Bypass valve: fully closed during the test,
  must be reopened a little so the LK can mix at cold starts (set it at the
  next cold start for a return of 55-58).

### 2026-10-05

- **Radio modules moved.** Both ISM_xSMART modules now sit in a junction box
  under the stairs, mid-house, on one 10 m Cat5 run from the ecoMAX G4 socket
  (D+/D- on one twisted pair, +12 V and GND each on a doubled pair; 13 V
  measured at the box). First attempt failed with both panels lost at once:
  the D+ conductor was not clamped at the controller end. Lesson: when both
  panels vanish together, it is the bus; when they vanish independently, it
  is radio.
- **P30 (radio signal) readings** after the move: Panel 1 77 %, Panel 3 65 %
  at the first panel positions; 100 % with a panel next to a module, 73 % at
  4 m through the floor. Panels then moved to spots giving 66–67 % each
  (15:40) to test whether equal signal gives equal dropout rates. Wi-Fi
  extender and switch next to the modules were powered off as a test: no
  change in P30.
- **Panel 3 temperature offset.** It read 3.6 K above Panel 1 side by side.
  Cause: user-menu parameter P32 (display correction) was +2.0. Set to -1.6.
- **Mixer 3 behaviour** (manual, eSTER_x40 DTR 1.2): the controller drops to
  "operation without a panel" a few minutes after losing radio contact; the
  thermostat lowers the mixer setpoint rather than stopping the pump. Whether
  a "pump off on thermostat" option exists in the ecoMAX service menu is still
  to be checked.
- **Telegraf stopped after a switch restart** at 15:44 and never reconnected
  (Pi kept publishing). Added `keepalive`/`ping_timeout` to the MQTT consumer
  and `scripts/telegraf-watchdog.ps1` as a 5-minute scheduled task. The
  planned nightly `wsl --shutdown` task is no longer needed; the watchdog only
  restarts when data actually stops.
- **Dashboard 5** gained the panel visibility lanes and the visibility %
  panel (dashboard version 26+).

### 2026-10-01 to 02

- Wood yield test, 9 kg apple wood, documented in the boiler notes (Pi repo
  CLAUDE.md): about 19–23 kWh delivered, 2.1–2.6 kWh/kg, roughly 5 kg of
  pellets. Wood mode at setpoint 72 with ~5 K hysteresis smoulders for hours;
  raise param 184 for wood burns. Burnout detection took 3 h and the loading
  pump cooled the buffer through the idle boiler meanwhile.
- Grafana: bar axes from zero, month names, bar alignment "after", boiler
  daily totals; mixer/ecoSTER fields shown as pump lanes.


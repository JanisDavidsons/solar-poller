# Exhaust fan controller at the boiler (spec for the Mac session)

Status: agreed with Janis on 2026-10-09, nothing built yet. Goal: load wood
without the command prompt. One touch on a screen at the boiler stops the
blower (DZT loop), runs the exhaust fan at 100 %, and afterwards steps the fan
back down on a timer. All levels and timers adjustable on the screen.

## What is already in place

- Atmos UCJ4C82 induced-draft fan on the flue, driven by the ecoMAX860P3-C
  exhaust-fan output (param 194 = 1). Wood-mode level is param 187 (currently
  30 after today's tests; 20 was too little, the rotor barely turns).
  Cleaning 156 = 100, burnout 159 = 80 stay with the controller.
- ecoNET 300 at http://192.168.8.3, basic auth admin:admin. Write:
  `GET /econet/rmNewParam?newParamIndex=187&newParamValue=100` →
  `{"paramIndex":"187","paramValue":100,"result":"OK"}`. Read back ~30 s
  later: `GET /econet/rmParamsData` → `.data[187].value`.
- DZT door input, terminals 48–49, low-voltage NC loop. Open loop = controller
  stops the blower (fanPower in regParams keeps showing 96, the API does not
  expose the door state; checked on all endpoints 2026-10-09). The exhaust fan
  is not affected by DZT. Today a manual switch sits in that loop; it goes
  away, the relay replaces it.
- Observed: at 100 % the underpressure pulls the boiler door shut. With the
  blower stopped the fire dies down at 20 % and at 30 % alike (flue gas
  −12…−20 K in 10 min), so the fan cannot feed the fire through a stopped
  blower; its job is draft at the door and after loading.
- Hardware on hand: Arduino Mega 2560 with MCUFRIEND_kbv parallel TFT shield
  and resistive touch, working code in
  https://github.com/JanisDavidsons/diesel_heater_interface (PlatformIO,
  `Display` class with pages and `Adafruit_GFX_Button`, touch calibration
  `TS_LEFT=927,TS_RT=63,TS_TOP=908,TS_BOT=125`, pins `XP=9,XM=A3,YP=A2,YM=8`,
  arduino-timer). Reuse `Display` (init, `TouchGetXY`, buttons, pages,
  draw-on-change helpers); drop `CanBusReceiver/Sender`, `RoomTemperature`,
  SD, DHT, RTC.

## Architecture (decided)

Two boards in one box at the boiler.

1. **Mega 2560 + TFT**: the panel and the logic. Owns the relay and the
   state machine. Works without the ESP (shows "no link", relay still works,
   fan level then cannot be written).
2. **Wemos D1 mini (ESP8266)**: network bridge on Mega `Serial1`
   (TX1 = 18, RX1 = 19). Writes param 187 to ecoNET over HTTP itself
   (the poller on the Pi is not touched), subscribes to the poller's MQTT
   topics for the numbers shown on screen, publishes the controller state.

Relay: one-channel 5 V relay module, **NC contact in the DZT loop**
(48–49). Coil off = loop closed = blower allowed. Mega reset, power loss or
hang therefore always lets the boiler run normally. Coil on only while
"loading".

Serial link: 3 wires, 9600 8N1. Mega TX1 (5 V) → resistor divider
(e.g. 2k2 / 3k3) → D1 RX; D1 TX (3.3 V) → Mega RX1 directly; common GND.
Power: one 5 V supply, Mega via VIN/USB and D1 via 5V pin (to decide: a
5 V USB adapter at the boiler, or a 12 V → 5 V buck from the boiler's 12 V).

## State machine (on the Mega)

Settings, all editable on screen and kept in EEPROM:

| name       | default | meaning                                              |
|------------|---------|------------------------------------------------------|
| `normal`   | 30 %    | standing level in wood mode (param 187) — test 0 later |
| `load`     | 100 %   | level while the door is open for loading             |
| `post`     | 40 %    | level after loading, for `post_min`                  |
| `post_min` | 30 min  | length of the post-load phase                        |
| `load_max` | 15 min  | safety: leave LOADING even if nobody presses "done"  |

States:

- **NORMAL**: relay off, 187 = `normal`.
- **LOADING** (big button "Lieku malku"): relay ON (blower stops), write
  187 = `load`. Screen shows a countdown of `load_max` and a button "Gatavs".
- **POST** ("Gatavs" pressed or `load_max` elapsed): relay off, write
  187 = `post`, countdown `post_min`, then back to NORMAL (write `normal`).
- Changing `normal` on screen while in NORMAL writes it immediately.
- Any write goes through the ESP; the Mega shows the value as "pending" until
  the ESP reports the read-back, then as confirmed. If no confirmation in
  60 s, show an error and retry once a minute.
- On boot: relay off, request 187 = `normal` once the ESP reports link.

Order of operations on "Lieku malku": relay first (blower stops at once),
then the fan write (takes effect within a few seconds, controller side).

### Draft follows the burn phase (added 2026-10-09 evening)

Observed: blower at 96 % the whole time, exhaust at 30 %, and the fire still
died down (flue 110 -> 68 in an hour); exhaust to 100 % brought it back in
4 minutes (68 -> 115). With the blower running the fire is draft-limited,
so the NORMAL level is phase-dependent, computed from the flue gas curve:

- main burn (flue > 90 and rising or flat): `normal_hi`, default 50 %
- ember phase (flue < 90 and falling slowly): `normal_lo`, default 30 %
- "fire dying" rule: wood loaded in the last `wood_hours` (default 6 h),
  blower running (`fan_works`), flue below 90 and falling more than 20 K
  in 20 min -> exhaust 100 % for 10 min. If the flue rises again, step back
  to the phase level; if not, send a phone alert (then the cause is the
  wood, not the draft). The motor protection rule stays on top.

`normal` in the state table above becomes these two values plus the rule;
the LOADING / POST levels are unchanged.

### How the ecoMAX decides the wood is burnt out (observed)

No fixed flue temperature. Both observed burn-outs fit: flue gas falls below
the boiler water temperature and stays there for param 185 ("Degvielas
trūkuma noteikšanas laiks", wood mode, 15 min); then Halted and, with
183 = 1, automatic switch to pellets (setpoint 80 -> 75, mode 5).
2026-10-11: flue below boiler from 00:18, fans off 00:34 at flue 58.5 /
boiler 61.9. 2026-10-09 morning: 32 min below before Halted, so the fit is
loose there. Param 180 "Ventilatora apstāšanās" = 5 (1-100, unit unknown,
read it on the controller screen) is probably the offset or delay of that
comparison; unconfirmed. Consequence with the exhaust fan: the sensor reads
lower (dilution), so burn-out is declared earlier and the embers are
abandoned without air. Panel rules to consider: exhaust to the minimum in
the ember phase, or the panel keeping wood mode alive while its own
estimate says the embers still deliver heat.

## Screen (320×240 landscape, reuse the page layout of the diesel project)

Main page: current fan level and state (NORMAL / LOADING 12:34 / POST 27:10),
flue gas °C, boiler °C, boiler mode text, link status (MQTT ok / no link),
age of the last MQTT value. Buttons: "Lieku malku" (large), "+ / −" for the
standing level, "Iestatījumi". Reload hint: when flue gas < `reload_c`
(default 78; Janis's refined rule 2026-10-12: load at flue 75-80, good
embers and the new wood catches fast; 65 was the earlier value) and the
boiler is in Operation or Supervision, show "Laiks likt malku" on the main
page; `reload_c` is a setting like the others.

Settings page: `normal`, `load`, `post`, `post_min`, `load_max` with +/−,
"Saglabāt". Keep the three-page structure (`page_0/1/2`) if convenient.

## Serial protocol Mega ↔ ESP (text lines, `\n` terminated)

Mega → ESP:
- `SET 100` request param 187 = 100
- `STATE <state> <relay> <level> <remaining_s>` every 5 s, e.g.
  `STATE LOADING 1 100 870` (ESP republishes to MQTT)

ESP → Mega:
- `ACK 187 100` after the HTTP write returned OK
- `VAL 187 100` after the read-back (~30 s later)
- `ERR 187 <text>` on HTTP failure
- `TEMP <flue> <co> <mode>` whenever MQTT delivers new values, e.g.
  `TEMP 78.6 67.7 3`
- `LINK 1|0` on MQTT connect/disconnect, also every 10 s as heartbeat

## ESP (D1 mini) behaviour

- Wi-Fi + MQTT to 192.168.8.50:1883 (anonymous), PubSubClient or similar.
- Subscribe `home/boiler/temp_flue_gas`, `home/boiler/temp_co`,
  `home/boiler/mode` (poller publishes bare numbers, retained).
- HTTP writes to ecoNET with basic auth; read-back 30 s after each write.
- Publish, bare numbers, retained, under `home/boiler/` so Telegraf files
  them in measurement `boiler` with the `metric` tag without config changes:
  `exhaust_set` (requested %), `exhaust_state` (0 NORMAL, 1 LOADING,
  2 POST), `dzt_open` (relay state 0/1), `wood_load` (publish `1` on each
  "Lieku malku" press, then `0` after 5 s — gives Grafana a marker per load).
- Add `exhaust_state` and `wood_load` lanes to dashboard 5 later; the lane
  query pattern is in README "Room panel visibility".

## Temperature sensors (decided 2026-10-09)

Two DS18B20 on one 1-Wire bus on the D1 mini (one GPIO, 4k7 pull-up),
published as bare numbers under `home/boiler/` every 30 s, retained:

- `temp_return`: boiler return pipe at the boiler inlet, after the LK
  mixing point, strapped to the pipe under insulation. Purpose: judge the
  manual bypass-valve position from Grafana. Reference readings 2026-10-09
  (analogue in-pipe thermometer): 67-68 with buffer bottom ~52 and boiler
  77, 61 with buffer bottom 60; a digital probe on the pipe surface read
  ~4 K lower. Floor: keep >= 55 at the start of a load.
- `temp_fan_motor`: exhaust fan motor housing, fixed to a housing screw or
  strapped with a spring clip. Readings so far: 42 °C at flue ~85 and 20 %,
  63 °C at flue ~100 and 30 %, 70 °C at flue ~110 and 30 % (2026-10-09
  evening, judged fine by Janis).

Both sensors are read by the ESP, forwarded to the Mega in the `TEMP` line
(extend it: `TEMP <flue> <co> <mode> <return> <motor>`), shown on the main
page.

Motor protection rule (on the Mega, settings like the others):
`motor_warn` (default 80 °C) and `motor_max` (default 90 °C, adjust once the
motor plate's insulation class / ambient rating is read). Above `motor_warn`
the panel shows a warning; above `motor_max` the Mega requests the fan at
100 % for `motor_cool_min` (default 5 min) regardless of state, since the
cooling disc moves more air at full speed, then returns to the state's
level and re-checks. If still above `motor_max` after two cycles, show an
alarm and keep 100 %. Direction confirmed by Janis: the fan has a small
cooling impeller on the motor shaft that moves almost nothing at low speed,
so full speed cools the motor.

## Fuel-mode parameter set (decided 2026-10-09, revised 2026-10-12)

**Revision 2026-10-12: this already exists.** The boiler_poller on the Pi has
a supervisor (`BUFFER_END_BY_FUEL`) that reads 161/162 every minute and
writes 162 per fuel after 3 stable cycles (it reverted a manual 72 to 62 at
15:11 on 2026-10-12). Decision: the supervisor stays the single owner of
fuel-mode parameters, wood 72 / pellets 50; the panel does NOT write 162 or
184/53, it only writes 187 and reads the rest. The poller is to publish
`buffer_end_target` and honour a retained override topic
`home/boiler/cmd/buffer_end` with an expiry, for tests and for the panel.
The table below is kept for reference only.


The buffer logic has one set of thresholds for both fuels (161 loading
start, upper sensor, 46; 162 loading end, lower sensor). Pellets should keep
62 (save pellets, deep cycles); wood should store as much as possible, so
162 = 72 while wood burns. The ESP applies the set when the fuel mode
changes (from the poller's `home/boiler/fuel_type`, or derived from
`temp_co_set` the same way):

| param | wood | pellets | note                                   |
|-------|------|---------|----------------------------------------|
| 162   | 72   | 62      | buffer loading end (lower sensor)      |
| 184   | 80   | -       | wood setpoint, already set             |
| 53    | -    | 75      | pellet setpoint, already set           |
| 187   | `normal` | -   | exhaust fan level, panel-controlled    |

Each write is read back and shown on the panel; the pellet set must be
restored before the controller runs pellets. Until the panel exists this is
done by hand (2026-10-09: 162 set to 72 for the evening wood burn, to be put
back to 62 before pellets).

## Failure behaviour

- ESP down / Wi-Fi down: relay still works; screen shows "no link"; fan stays
  at whatever the controller has. Acceptable: loading still possible, just
  without the 100 % boost.
- ecoNET write fails: retry once a minute, show error; never block the relay.
- Mega down: relay NC → blower allowed, fan at last written level.
- Param 187 is wood mode only. In pellet mode the exhaust levels are params
  6/7/8 (max/mid/min) and the device must not touch them; the Mega has no
  fuel-mode input, so the ESP should refuse writes when `mode`/fuel type
  from MQTT says pellets (check `temp_co_set` vs the stored wood/pellet
  setpoints the same way the poller derives `fuel_type`, or subscribe to
  `home/boiler/fuel_type` if the poller publishes it).

## Repo

New PlatformIO project (suggested `~/Projects/boiler-panel`), two envs:
`[env:megaatmega2560]` and `[env:d1_mini]` with `src_dir` per env, or two
folders. Copy `Display.*` from the diesel repo and strip the heater fields.

## Phase 2 ideas (after phase 1 has run for a week)

- Heat sink on demand: boiler > ~82 with the buffer at its loading-end
  limit -> raise Mixer 3 setpoint (param 74) for 20-30 min so the upstairs
  radiators absorb the surplus, then restore. Same mechanism the controller
  triggers by accident when room panel 3 drops out, but deliberate and
  time-limited.
- Earlier loading start when the house is cold and the buffer empty
  (raise 161 temporarily).
- Flap on the Ø40 dome port driven by a Regin RDAB5-230 damper actuator
  (on hand): 5 Nm, 100-240 V AC, 3-point (1 = N, 2 = L one way, 3 = L the
  other), internal end stops, 150 s per 90°, no spring return, no aux
  contact, manual release button. ONE 230 V relay channel on the HL-56S,
  using the SPDT contact: L to COM, NC to actuator terminal 2 (close), NO to
  terminal 3 (open); relay released = flap closed and held, energised = open,
  no interlock needed and power-off = closed. (Same trick the previous owner
  used with an Omron H3DS-ML timer's 15/16/18 contact on this actuator.)
  Position known by time. The stroke is limited mechanically with the two
  slot stops on the actuator to ~45° (about 75 s); allow 85 s before
  releasing the "open" timing. Mount on a bracket 15-20 cm off the boiler
  body with a rod linkage (ambient limit +50 °C). Open the
  flap on the reload hint (flue below 75-80) so it is already open when
  "Lieku malku" is pressed; close on "Gatavs". Publish `damper_state`.
- Every rule: explicit trigger, time limit, restore path, and a published
  `home/boiler/rule_active` code so Grafana shows which rule did what.

## Roadmap (agreed 2026-10-09)

Direction: the ecoMAX keeps the pellet burner (feeder, ignition, flame
control, burn-back protection, STB) and the hydraulics. Janis's controller
takes over, step by step and each step reversible, the decisions the ecoMAX
makes poorly or not at all. Everything is tested in steps, and each step is
judged on measured results (Influx), not on expectations.

Phase 1, the panel (this document): loading sequence, exhaust fan levels,
motor protection, fuel-mode parameter set, two DS18B20, reload hint.

Phase 2, supervisor rules through parameters, no new wiring:
- Room panel 3 dropout: hold Mixer 3 setpoint (param 74) at a sane value
  while `room_temp3` is missing, restore when it returns. Fixes the hot
  upstairs radiators that started this whole project.
- Heat sink on demand (radiators) and earlier loading start, see Phase 2
  ideas.
- Own room sensors instead of the eSTER radio link: the ecoMAX accepts a
  plain room-thermostat contact per mixer ("Termostata izvēle" params
  134-139, lowering "Telpas termost. mais N" 87-91). A relay on the panel
  (or a second ESP upstairs) plays that thermostat, driven by sensors that
  work over Wi-Fi/Zigbee; the eSTER panels stay as displays. Removes the
  panel-dropout problem (4 m of distance already loses the eSTER link)
  instead of fighting it.
- Alerts to the phone: feeder temperature trend, flue gas > 250 and rising
  fast (chimney fire), boiler > 90, data gone stale, smoke detector input.
- Loading assistant: not only "time to load" but how much: buffer headroom
  plus expected draw for the next hours, converted to kg with the measured
  2.1-2.6 kWh/kg; every load logged, yield computed automatically.
- One policy across the three energies: wood, pellets, PV surplus into the
  buffer heating elements (solar-poller already switches them); buffer
  thresholds per fuel, DHW priority, forecast-aware (do not fill the buffer
  with pellets in the evening when tomorrow is sunny).

Phase 3, own combustion control in wood mode:
- Blower on Janis's own output (SSR or PWM), driven by flue gas, buffer
  headroom and, if the lambda module is present (param 191, moduleLambdaSoftVer),
  O2; full power while the buffer has room, a lean ember phase instead of the
  ecoMAX's 96 % / Supervision on-off. ecoMAX keeps pumps and mixers.
- Before taking over: shadow mode for about a month, the new logic reads the
  same data and only logs what it would have done; compared against what the
  ecoMAX did, especially in the moments something went wrong.
- Success measure: kWh/kg per load (the wood yield test), door smoke, number
  of Supervision cycles per load.

## Hardware on the bench (2026-10-11, supersedes the power/ESP lines above)

- PSU Mean Well RS-15-5, 5 V / 3 A, FG to PE. Measure the output unloaded
  first (target 5.0-5.1 V, trim pot). Feeds Mega `5V` pin, NodeMCU `VIN`,
  relay board `VCC`/`JD-VCC`. Total draw ~1 A. Disconnect the PSU whenever
  a board is plugged into USB for flashing (Mega 5V pin and NodeMCU VIN are
  both tied to the USB 5 V rail without a known diode).
- Relay board HL-56S V1.0, 6x SRD-05VDC-SL-C, opto inputs, active LOW,
  JD-VCC jumper stays (driven from Mega 5 V logic, so no 3.3 V issue).
  Channel 1: NC contact in the DZT loop 48-49. Keep that low-voltage channel
  away from any channel that will later switch 230 V (pump vetoes).
- NodeMCU LoLin V3 (ESP-12F, CH340) instead of the D1 mini: same role,
  network bridge on Mega Serial1. D pins are 3.3 V: Mega TX1 -> divider
  (2k2/3k3) -> ESP RX; ESP TX -> Mega RX1 direct; common GND.
- Mega 2560 + 3.5" MCUFRIEND TFT shield (UNO pin layout, 8-bit parallel).
  Pins taken by the shield, from `diesel_heater_interface/src/main.cpp` and
  the MCUFRIEND_kbv shield defaults: data D2-D9, control A0-A4
  (RD, WR, RS, CS, RESET), touch XP=9, XM=A3, YP=A2, YM=8 (shared with
  display lines, restored after each read as in `Display::TouchGetXY`),
  SD CS D10 (unused now), init `tft.readID()` then `tft.begin(ID)`,
  `setRotation(1)`, calibration TS_LEFT=927, TS_RT=63, TS_TOP=908,
  TS_BOT=125. MCP2515 on D53/SPI is dropped.
- Free Mega pins for this project: Serial1 D18/D19 (ESP), relays D22-D27
  (IN1-IN6), physical button D30 with internal pull-up, 1-Wire bus D32
  with 4k7 to 5 V (both DS18B20), spare D34-D49, A5-A15; D20/D21 (I2C)
  stay free for a later I2C device. D50-D53 free since the CAN module is
  gone.
- DS18B20: boiler inlet after the LK 820, and the exhaust fan housing
  (max seen 70 °C, sensor limit 125 °C; silicone or PTFE cable).
- Loading timer `load_max`: Janis wants 2-3 min, not 15; make it a setting,
  default 3 min, and it must return by itself when "Gatavs" is forgotten.
- Project folder: `~/Projects/veto-relay/` proposed on 2026-10-11 (the
  2026-10-09 suggestion was `boiler-panel`); either name, one clean repo.
- Bench test order: (1) PSU unloaded; (2) Mega alone, relay test sketch,
  each channel in turn, confirm all release on reset and stay released
  while the Mega boots (inputs float high, opto off); (3) touch + display
  from the diesel code with CAN removed; (4) ESP serial link; (5) sensors.

## Open items

- Power source at the boiler (USB adapter vs 12 V buck).
- Whether `normal` ends up 0: run the test "blower on, exhaust off 15 min,
  smell at the door" once the panel exists.
- Motor temperature at higher flue temps (42 °C at ~85 °C flue so far).
- Resolved 2026-10-09: two DS18B20 sensors are in scope (see "Temperature
  sensors" below), not optional.

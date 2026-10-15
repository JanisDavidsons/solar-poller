# veto-relay firmware: handoff for the Mac Claude Code session (2026-10-13)

Goal: replace the bench firmware in `~/Projects/veto-relay` with the real
phase-1 firmware for both boards. Wiring is done and bench-tested (relays,
TFT, touch, Mega<->ESP serial). DS18B20 sensors are NOT wired yet.

Read first, in this order:
1. `docs/exhaust-fan-controller.md` (the spec; copy it into the veto-relay
   repo as `docs/exhaust-fan-controller.md`). Where sections disagree, the
   later dated sections win: "Bench results 2026-10-13" and "Hardware on
   the bench 2026-10-11" override the older Architecture / Screen /
   Temperature-sensor text. Resolved list below.
2. `~/Projects/veto-relay/src/main.cpp` (bench firmware): the touch
   handling there is measured and works; reuse it as is.
3. `~/Projects/solar-poller/boiler_poller.py`: MQTT topic names and
   encodings (`mode`, `fuel_type`, `grate_mode`, `exhaust_level`,
   `temp_flue_gas`, `temp_co`, `temp_upper_buf`, `temp_lower_buf`,
   `fan_works`). Do not change the poller.

## Repo layout (keep)

- Root PlatformIO project = Mega (`[env:mega]`, `src/`). `Display.*` from the
  diesel project may be reused or replaced; CAN, RoomTemperature, SD, DHT,
  RTC sources and libs can be removed.
- `esp/` = separate PlatformIO project for the NodeMCU (`board = nodemcuv2`).
- WiFi credentials in `esp/src/secrets.h`, gitignored, with a
  `secrets.example.h` committed.
- Add a short `CLAUDE.md` in the repo with the pin map and the rules below.

## Resolved decisions (override older spec text)

- Boards: Mega 2560 + MCUFRIEND 3.5" TFT (on wires), NodeMCU LoLin V3
  (not a D1 mini), HL-56S 6-ch relay board (active LOW), RS-15-5 PSU.
- Screen is **400x240** landscape (`setRotation(1)`), not 320x240.
- Mega pins: Serial1 = 18 (TX1) / 19 (RX1), 9600 8N1 (clone silkscreen
  shows TX/RX swapped; the core numbering is right). Relays: screen
  R1..R6 = `{32, 30, 28, 26, 24, 22}` (header wired from the far end).
  R1 = DZT loop (COM+NC, coil on = blower stopped). R6 = Regin flap
  (phase 2, leave unused but reserved). R2..R5 spare (pump vetoes later).
  1-Wire bus for both DS18B20 on Mega **D34** (4k7 to 5 V); the sensors are
  read by the Mega, not the ESP. Optional hardware button D36 (pull-up).
- ESP: `Serial.begin(9600); Serial.swap();` -> D7 = RX, D8 = TX. No USB
  serial after swap; debug only via `Serial1` on D4 if needed.
- Relay safety: all relay pins `digitalWrite(HIGH)` BEFORE `pinMode(OUTPUT)`
  in setup. Never switch two relay coils in the same instant (stagger by
  >= 300 ms): doing so coincided with ESP link loss on the bench.
- `load_max` default **3 min** (Janis wants 2-3 min), not 15.
- The panel writes **only param 187** (exhaust fan, wood mode). It never
  writes 162 (the poller supervisor owns it), 184, 53 or pellet exhaust
  params 6/7/8. The ESP must refuse 187 writes unless the boiler is in wood
  mode (decide from `grate_mode` / `fuel_type` per boiler_poller.py;
  `fuel_type` flickers when param 68 raises the setpoint, so require it
  stable, e.g. same value for 3 minutes, or prefer `grate_mode` if the
  poller defines it as the grate/wood flag; check, do not guess).
- Read-back of 187: the poller already publishes `home/boiler/exhaust_level`
  every minute (retained). The ESP may use that as the confirmation instead
  of its own HTTP read-back, or do both; state which in CLAUDE.md.
- Serial protocol: as in the spec (`SET`, `STATE`, `ACK`, `VAL`, `ERR`,
  `TEMP`, `LINK`), plus: the Mega sends sensor values to the ESP for
  publishing, e.g. `SENS <return> <motor>` every 30 s (`nan` when a sensor
  is absent). Both sides ignore non-printable bytes (Mega TX1 emits a null
  byte at boot) and lines longer than 80 chars.
- MQTT: broker 192.168.8.50:1883, anonymous. Publish bare numbers, retained,
  under `home/boiler/`: `exhaust_set`, `exhaust_state` (0/1/2),
  `dzt_open` (0/1), `wood_load` (1 on each load press, 0 after 5 s),
  `temp_return`, `temp_fan_motor` (only when the sensor reads valid).
  Telegraf files them without config changes.
- ecoNET: `http://192.168.8.3`, basic auth admin:admin,
  `GET /econet/rmNewParam?newParamIndex=187&newParamValue=N`.

## Touch (measured, keep exactly)

From bench `main.cpp`: `getPoint()` twice, use the second; pressure window
`z > 100 && z < 1000` (diesel's 200 ignores the right half of this panel);
restore `YP`/`XM` as outputs HIGH after each read; a press counts only when
two consecutive reads hit the same button; after an action nothing counts
until 300 ms with no touch at all. Buttons are initialised once per page
draw and never re-initialised in a handler; state is shown by drawing next
to/under the button. Calibration TS_LEFT=927, TS_RT=63, TS_TOP=908,
TS_BOT=125. Buttons at least 120x48 px. Redraw only what changed, no
full-area `fillRect` on a timer.

## Scope of phase 1 (build this)

1. State machine NORMAL / LOADING / POST with `normal`, `load`, `post`,
   `post_min`, `load_max` in EEPROM, editable on a settings page.
   LOADING: R1 on first, then `SET <load>`; ends on "Gatavs" or `load_max`.
   POST: R1 off, `SET <post>` for `post_min`, then `SET <normal>`.
   Boot: R1 off, `SET <normal>` once the ESP reports link and wood mode.
2. Main page: state + countdown, current requested / confirmed level,
   flue gas, boiler, buffer top/bottom, mode text, link status and age of
   the last MQTT value, large "Lieku malku" button, + / - for `normal`,
   "Iestatījumi". Reload hint "Laiks likt malku" when flue < `reload_c`
   (default 78) and mode is Operation or Supervision.
3. Pending / confirmed / error display for 187 writes; retry once a minute
   on failure; the relay never waits for the network.
4. DS18B20 support on D34 that runs cleanly with zero sensors attached.
   Motor protection rule from the spec (`motor_warn` 80, `motor_max` 90,
   `motor_cool_min` 5) active only when the motor sensor is present.
5. ESP: WiFi + MQTT reconnect without blocking the serial bridge; `LINK`
   heartbeat every 10 s; wood-mode guard; publishing as above.

Not in phase 1 (leave hooks, do not implement): draft-follows-burn-phase
and the "fire dying" rule, ember-phase rule, Regin flap on R6, pump
vetoes, phone alerts.

## Failure behaviour (must hold)

Mega reset, hang or power loss -> R1 released -> DZT loop closed -> blower
allowed. ESP or WiFi down -> relay still works, screen shows "no link".
No code path may leave R1 energised longer than `load_max`.

## Working rules with Janis

- Ask instead of assuming; never present a guess as fact.
- Janis commits and pushes himself; prepare changes, do not commit.
- Compile both projects (`pio run`, `cd esp && pio run`) before handing
  over. Janis flashes: ESP first, then Mega, PSU off while USB is plugged.
- Nothing may raise pellet consumption; the panel touches wood mode only.
- Before the panel goes on the live boiler: Janis verifies with a
  multimeter that screen R1 opens the DZT wires (not yet confirmed).

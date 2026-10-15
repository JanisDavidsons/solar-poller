# Bench build inside diesel_heater_interface

1. Copy `main_bench.cpp` to `diesel_heater_interface/src/`.
2. Wrap the existing `src/main.cpp` in `#ifndef BENCH` ... `#endif`
   (first and last line of the file).
3. Add to `platformio.ini`:

```
[env:bench]
platform = atmelavr
board = megaatmega2560
framework = arduino
build_flags = -DBENCH
lib_deps = ${env:megaatmega2560.lib_deps}
upload_port = /dev/cu.usbserial-14310
monitor_speed = 115200
```

4. `pio run -e bench -t upload` then `pio device monitor -e bench`.
   The old env still builds the diesel firmware unchanged.

ESP side: `../esp_bench/esp_bench.ino` (Arduino IDE or a second PlatformIO
env with `board = nodemcuv2`). Flash the ESP first, then the Mega.

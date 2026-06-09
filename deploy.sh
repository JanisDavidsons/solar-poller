#!/bin/bash
# Deploy solar_poller (C++) and boiler_poller (Python) from this Mac repo
# to the OrangePi. Builds the C++ binary on the Pi (architecture-matched),
# installs it, and restarts both services.

set -e   # bail on first error

PI="pi"
SRC="solar_poller.cpp"
SRC_PY="boiler_poller.py"

echo "==> Syncing $SRC to $PI:/home/janis/"
rsync -avz "$SRC" "$PI:/home/janis/"

echo "==> Syncing $SRC_PY to $PI:/home/janis/"
rsync -avz "$SRC_PY" "$PI:/home/janis/"

echo "==> Building on Pi"
ssh "$PI" 'g++ -O2 -Wall -o /home/janis/solar_poller /home/janis/solar_poller.cpp -lmodbus -lmosquitto -lgpiod'

echo "==> Stopping solar-poller"
ssh "$PI" 'sudo /usr/bin/systemctl stop solar-poller'

echo "==> Installing new binary"
ssh "$PI" 'sudo /usr/bin/cp /home/janis/solar_poller /usr/local/bin/solar_poller'

echo "==> Starting solar-poller"
ssh "$PI" 'sudo /usr/bin/systemctl start solar-poller'

echo "==> Restarting boiler-poller"
ssh "$PI" 'sudo /usr/bin/systemctl restart boiler-poller'

echo "==> Recent service logs"
sleep 2
ssh "$PI" 'journalctl -u solar-poller -n 10 --no-pager && echo "---" && journalctl -u boiler-poller -n 5 --no-pager'

echo "==> Done"

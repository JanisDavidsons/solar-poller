#!/bin/bash
# Deploy solar_poller from this Mac repo to the OrangePi.
# Builds on the Pi (architecture-matched), installs the binary, restarts the service.

set -e   # bail on first error

PI="pi"
SRC="solar_poller.cpp"

echo "==> Syncing $SRC to $PI:/home/janis/"
rsync -avz "$SRC" "$PI:/home/janis/"

echo "==> Building on Pi"
ssh "$PI" 'g++ -O2 -Wall -o /home/janis/solar_poller /home/janis/solar_poller.cpp -lmodbus -lmosquitto -lgpiod'

echo "==> Stopping service"
ssh "$PI" 'sudo /usr/bin/systemctl stop solar-poller'

echo "==> Installing new binary"
ssh "$PI" 'sudo /usr/bin/cp /home/janis/solar_poller /usr/local/bin/solar_poller'

echo "==> Starting service"
ssh "$PI" 'sudo /usr/bin/systemctl start solar-poller'

echo "==> Recent service logs"
sleep 2
ssh "$PI" 'journalctl -u solar-poller -n 15 --no-pager'

echo "==> Done"

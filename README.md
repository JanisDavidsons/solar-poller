# solar-poller

Home solar energy management system. Polls an Eastron SDM630 energy meter
via Modbus RTU, publishes per-phase power data to MQTT, and drives a hot
water heater via SSR when solar surplus is available.

Runs on an OrangePi One under Armbian Trixie. See `project-summary.md`
for full architecture and operational notes.

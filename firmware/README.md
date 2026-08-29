# MicoAir F405 V2 firmware

This firmware turns the MicoAir F405 V2 BMI088/SPL06 flight controller into
GuessWork's camera-trigger and IMU clock master. It is intentionally not a
flight-control firmware.

Build:

```sh
pio run -e micoair_f405_v2
```

Flash from STM32 ROM DFU mode:

```sh
pio run -e micoair_f405_v2 -t upload
```

Key modules:

- `include/sync_controller_protocol.h`: shared host/firmware USB contract;
- `src/timebase.cpp`: TIM2 1 MHz 64-bit extended clock;
- `src/trigger_engine.cpp`: interrupt-scheduled M1–M6 pulse groups;
- `src/bmi088.cpp`: onboard SPI2 sensor initialization and 400 Hz reads;
- `src/main.cpp`: USB command parser and multiplexed telemetry.

The board boots with every output LOW and stopped. It only produces periodic
triggers after an atomic `SET_CONFIG` and `ARM`. Loss of the USB CDC host
stops outputs. Configuration is not persisted on the board; the host replays
database intent after discovery/reconnect.

See `docs/micoair-f405-v2.md` for pin mappings, power caveats, camera wiring,
and physical acceptance tests.


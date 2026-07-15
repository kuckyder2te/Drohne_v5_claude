# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a **PlatformIO** project targeting the **Raspberry Pi Pico** (RP2040) with the Earle Philhower Arduino core.

```bash
# Build
pio run

# Upload to device
pio run --target upload

# Monitor serial output (COM17 @ 115200 baud, see platformio.ini)
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor
```

There is no automated test suite — validation happens via the hardware test modes below plus manual bench/flight testing (see README.md "Ein-/Ausschalten" for the arm/disarm power-on procedure).

Build flags in `platformio.ini`: `-DGLOBAL_DEBUG` is always on. Test modes are enabled by uncommenting exactly one `#define` in [include/config.h](include/config.h); leave all commented for normal flight operation.

Logging is controlled by two flags consumed in [include/myLogger.h](include/myLogger.h): `_SERIAL_LOG` (USB Serial) and `_BT_LOG` (Bluetooth). Use `LOG(msg)` and `LOG_FMT(fmt, ...)` — they route to both outputs simultaneously when enabled.

**No new libraries without checking with the user first** (project rule from README.md).

## Architecture Overview

The firmware implements a **cascaded PID altitude + attitude stabilizer** for a quadcopter running on RP2040. All logic lives in `src/main.cpp`'s `setup()`/`loop()`, guarded by `#ifdef` blocks per test mode — exactly one code path is active at a time, selected at compile time via `config.h`.

### Control Loop (normal operation, all `TEST_*` macros undefined)

1. **Sensor update** every loop iteration — barometer, ultrasonic, IMU all polled unconditionally.
2. **Safety checks** — disarms (`disarm()`) on IMU-not-ready or an altitude jump > 500 cm between consecutive loop iterations.
3. **Input processing** — a single active channel, `comm` in `main.cpp`, drives all key/command handling. `bt` and `serial` are both `CommChannel` instances created in `setup()` after their respective `Stream` is fully configured (`Serial1`/BT UART and `Serial`/USB); `comm` is a pointer assigned to whichever one should actually be used, switchable by changing one line in `setup()`. PID-tuning strings go through `CommChannel::processCommand()`.
4. **Arm sequence** — pressing `a` twice within 3 s recalibrates the barometer, resets all three PID controllers, arms, and sets `targetHeightCm = 20`. `d` disarms immediately (motors to `ESC_MIN_US`).
5. **Height PID** (every `PID_INTERVAL_MS` = 50 ms) — error = `targetHeightCm − currentAltitude`; altitude source is ultrasonic when valid, barometer otherwise. Output includes the `THROTTLE_OFFSET_US` baseline.
6. **Anti-windup / liftoff gating** — the integral term on all three PID controllers only accumulates once `ultrasonic.isValid() && altitude > LIFTOFF_HEIGHT_CM` (`enableIntegral()`); it's cleared again on landing. This prevents integral windup while sitting on the ground pre-liftoff.
7. **Attitude PID** — roll/pitch corrections from IMU angles, target = 0°; output is a pure ±500 correction (no offset).
8. **Motor mixing** — X-configuration in `MotorMixer::mix()`: `FL = throttle − roll + pitch`, `FR = throttle + roll + pitch`, `BL = throttle − roll − pitch`, `BR = throttle + roll − pitch`. Depends only on motor position, not physical spin direction (fixed mechanically by ESC wiring) — see README.md "Drehrichtungen" for the CW/CCW mapping and prop-pitch safety note.
9. **Status log** every 500 ms, toggled at runtime with `l`.

### Module Map

| Path | Responsibility |
|------|---------------|
| [include/config.h](include/config.h) | All tunable parameters: ESC limits, PID defaults, flight parameters, test-mode switches |
| [include/pins.h](include/pins.h) | Single source of truth for all GPIO assignments |
| [include/myLogger.h](include/myLogger.h) | `LOG`/`LOG_FMT` macros — dual output to USB Serial + BT UART |
| [src/control/MotorMixer.cpp](src/control/MotorMixer.cpp) | PWM to ESCs using native RP2040 `hardware/pwm.h` SDK (50 Hz, 20000 wrap, 1000–2000 µs) |
| [src/control/PIDController.cpp](src/control/PIDController.cpp) | Custom PID — `useOffset=true` (height) adds `THROTTLE_OFFSET_US` so output is absolute throttle clamped to `[ESC_MIN_US, ESC_MAX_US]`; `useOffset=false` (roll/pitch) outputs a pure ±500 correction; integral only accumulates while `enableIntegral(true)` |
| [src/sensor/IMU.cpp](src/sensor/IMU.cpp) | ICM-20948 9-DoF via the `wollewald/ICM20948_WE` library at I2C address **0x69** (board-specific quirk — datasheet implies 0x68 for AD0=GND); complementary filter (`alpha=0.98`) fusing gyro integration with accel-derived roll/pitch; I2C bus recovery via 9 SCL pulses in `main.cpp::i2cBusRecovery()` before `Wire.begin()` |
| [src/sensor/Barometer.cpp](src/sensor/Barometer.cpp) | MS5611 (0x77); needs a 90 s warmup + calibration before it's trustworthy; ring-buffer filter; `BARO_TEMP_COEFF` compensates thermal drift |
| [src/sensor/Ultrasonic.cpp](src/sensor/Ultrasonic.cpp) | HC-SR04 on pins 8/6; valid range ~2–300 cm; preferred altitude source over barometer whenever `isValid()` |
| [src/sensor/Battery.cpp](src/sensor/Battery.cpp) | ADC pin 26, voltage divider; warns/critical via buzzer pin 10 |
| [src/comm/CommChannel.cpp](src/comm/CommChannel.cpp) | Transport-agnostic key/command parser and PID-tuning command processor. Constructor takes any `Stream&` (used for BT UART `Serial1` and USB `Serial` today; a TCP `Client` would work too, since it also derives from `Stream`). ANSI escape sequences are always discarded; `+`/`-` act immediately; other single-char commands (`A D H R L S ?`) resolve after a 200 ms timeout or on newline (whichever comes first); multi-char strings (`RP=`/`RI=`/`RD=`, `PP=`/`PI=`/`PD=`, height `P=`/`I=`/`D=`, `SAVE`, `RESET`) terminate on newline only. |
| [src/storage/Settings.cpp](src/storage/Settings.cpp) | EEPROM persistence for height PID Kp/Ki/Kd, validity marker byte |

### Test Modes

Defined in [include/config.h](include/config.h). Uncomment exactly one to run that test; leave all commented for normal flight operation:

- `TEST_MOTORS` — all four motors together, plus an ESC-calibration sub-sequence (`c`/`k`/`m`)
- `TEST_MOTORS_SINGLE` — drive one motor by index (`1`=FL, `2`=FR, `3`=BR, `4`=BL)
- `TEST_BAROMETER` — continuous pressure/altitude/temperature print
- `TEST_KEYBOARD` — BT/keyboard command echo + barometer print
- `TEST_I2C_SCAN` — scans the I2C bus every 5 s (expects `0x69` IMU, `0x77` baro); includes an SDA-stuck-low hardware-fault check before scanning
- `TEST_IMU` — continuous roll/pitch/AccZ print
- `TEST_ULTRASONIC` — HC-SR04 distance print every 200 ms

### Key Design Decisions

- **ICM-20948 IMU via `ICM20948_WE` library** — replaced a previous custom MPU9250 I2C driver, driven by repeated ESD failures of MPU9250 boards (see README "Sicherheit & Handhabung"). On this specific board the sensor answers at I2C address 0x69 even with AD0 tied to GND, not the datasheet's 0x68 — hardcoded in `IMU.h`; don't "fix" it back to 0x68.
- **Native RP2040 PWM SDK** (`hardware/pwm.h`) instead of `RP2040_PWM` library — more stable, no library dependency.
- **Custom PID** instead of FastPID — FastPID's coefficient clamping conflicted with required ranges.
- **Ultrasonic preferred over barometer** when in range (2–300 cm) — better accuracy and no warmup requirement.
- **I2C bus recovery** — sends 9 clock pulses to release a stuck SDA line before every `Wire.begin()`, both in normal operation and in `TEST_I2C_SCAN`.
- **Integral anti-windup gated on liftoff** (`LIFTOFF_HEIGHT_CM`) — height/roll/pitch integrators stay at zero until the ultrasonic confirms the craft is airborne, and are cleared again on landing.
- **Single active input/log channel via a `comm` pointer** — `bt` and `serial` are both `CommChannel` instances, only constructed in `setup()` once their underlying `Stream` (`Serial1`/BT UART, `Serial`/USB) has been fully configured (pins + `begin()`). A single `CommChannel* comm` pointer selects which one is actually used for input and logging; there is no BT-primary/Serial-fallback redundancy — switching channels means changing the `comm = ...` assignment in `setup()`. Both channels share one `CommChannel` class (parameterized by `Stream&`) with a unified parsing scheme: ANSI arrow-key escape sequences are always discarded (harmless no-op for BT), single-char commands resolve via a 200 ms timeout so BT apps that don't send Enter still work, and `+`/`-` act instantly. Note: `d` (disarm) is not byte-instant — it resolves within ≤200 ms — because a byte-instant `d`/`D` would break the `D=<value>` (height Kd) tuning command the instant the `D` byte is read, before `=<value>` arrives.
- **Yaw = 0 currently** — gyro-based yaw stabilization deferred to Phase 3; only the complementary-filtered roll/pitch are used for attitude control.

### Planned Phases (not yet implemented)

- **Phase 2**: NRF24L01 remote control (library already in `platformio.ini`, SPI pins defined in `pins.h`, not yet wired into `main.cpp`)
- **Phase 3**: Yaw stabilization using ICM-20948 magnetometer + full gyro integration
- **Phase 4**: GPS-assisted position hold
- **Phase 5**: Autonomous flight routes

### Hardware Notes

See README.md for full hardware detail (pinout table, motor spin-direction verification, ESD handling procedure, power-on/off sequencing). Highlights relevant to code changes:

- **MS5611 not found**: check PS/NCS pins on the CJMCU-10DOF-style board are tied to 3.3 V; run `TEST_I2C_SCAN`.
- **Barometer drift indoors**: needs the full 90 s warmup and a recalibration (`r`) immediately before arming.
- **Arrow keys not recognized over USB**: use `pio device monitor`, not the VS Code built-in Serial Monitor — and remember BT is the intended primary input anyway.
- **Pico not detected by picotool**: hold BOOTSEL, flash `flash_nuke.uf2`, then re-flash normally.
- **ICM-20948 not found despite correct wiring**: confirm it enumerates at 0x69, not 0x68, via `TEST_I2C_SCAN`.
- **Motor spin direction vs. propeller pitch**: the CW/CCW assignment in README is specific to this physical board's ESC wiring, not a universal rule — always verify per-motor with `TEST_MOTORS_SINGLE` (props off) before mounting propellers, and match propeller pitch (normal vs. pusher) to the observed direction.

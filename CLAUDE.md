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

# Build/flash one standalone hardware test tool (see Test Modes below),
# e.g. the IMU tool under test/test_imu/ — main.cpp is NOT compiled for this
pio test -e rpipico -f test_imu --without-testing
pio device monitor
```

There is no automated test suite — validation happens via the hardware test tools below plus manual bench/flight testing (see README.md "Ein-/Ausschalten" for the arm/disarm power-on procedure).

Build flags in `platformio.ini`: `-DGLOBAL_DEBUG` is always on. `NORMALBETRIEB` in [include/config.h](include/config.h) is the master switch: while defined (default) it force-`#undef`s `TEST_KEYBOARD`, so normal flight operation is active regardless of whether it's left uncommented below. To run the keyboard test, comment out `NORMALBETRIEB` and uncomment `TEST_KEYBOARD`. The other six former `TEST_*` modes are not `config.h` switches anymore — they're standalone tools under `test/`, selected via `pio test -f <name>` (see Test Modes below), never `NORMALBETRIEB`/`main.cpp`.

Logging is controlled by two flags consumed in [include/myLogger.h](include/myLogger.h): `_SERIAL_LOG` (USB Serial) and `_BT_LOG` (Bluetooth). Use `LOG(msg)` and `LOG_FMT(fmt, ...)` — they route to both outputs simultaneously when enabled.

**No new libraries without checking with the user first** (project rule from README.md).

## Architecture Overview

The firmware implements a **cascaded PID altitude + attitude stabilizer** for a quadcopter running on RP2040. Normal-operation orchestration lives in `src/main.cpp`'s `setup()`/`loop()`, guarded by `#ifdef NORMALBETRIEB`, but the actual flight state and control logic (arm/disarm, safety checks, PID + motor mixing, status log) is encapsulated in [src/control/FlightController.cpp](src/control/FlightController.cpp) — `main.cpp` just polls sensors each iteration and calls into `FlightController`/`InputHandler`. The one remaining in-firmware test mode (`TEST_KEYBOARD`) lives in [src/testmode/TestModes.cpp](src/testmode/TestModes.cpp), mutually exclusive with `NORMALBETRIEB` via `config.h`. The other six hardware test tools are separate standalone programs under `test/` (see Test Modes below) — they never touch `main.cpp`.

### Control Loop (normal operation, `NORMALBETRIEB` defined)

1. **Sensor update** every loop iteration — barometer, ultrasonic, IMU all polled unconditionally in `main.cpp::loop()`.
2. **Safety checks** — `FlightController::checkSafety()` disarms (`FlightController::disarm()`) on IMU-not-ready or an altitude jump > 500 cm between consecutive loop iterations (based on barometer altitude).
3. **Input processing** — a single active channel, `comm` in `main.cpp`, drives all key/command handling via [src/control/InputHandler.cpp](src/control/InputHandler.cpp). `bt` and `serial` are both `CommChannel` instances created in `setup()` after their respective `Stream` is fully configured (`Serial1`/BT UART and `Serial`/USB); `comm` is a pointer assigned to whichever one should actually be used, switchable by changing one line in `setup()`. PID-tuning strings go through `CommChannel::processCommand()`, called against the `PIDController` instances owned by `FlightController` (`getPidHeight()`/`getPidRoll()`/`getPidPitch()`).
4. **Arm sequence** — `FlightController::requestArm()`: pressing `a` twice within 3 s recalibrates the barometer, resets all three PID controllers, arms, and sets `targetHeightCm = 20`. `d` calls `FlightController::disarm()` immediately (motors to `ESC_MIN_US`).
5. **Height PID** (every `PID_INTERVAL_MS` = 50 ms, in `FlightController::updateControlLoop()`) — error = `targetHeightCm − currentAltitude`; altitude source is ultrasonic when valid, barometer otherwise. Output includes the `THROTTLE_OFFSET_US` baseline.
6. **Anti-windup / liftoff gating** — the integral term on all three PID controllers only accumulates once `ultrasonic.isValid() && altitude > LIFTOFF_HEIGHT_CM` (`enableIntegral()`); it's cleared again on landing. This prevents integral windup while sitting on the ground pre-liftoff.
7. **Attitude PID** — roll/pitch corrections from IMU angles, target = 0°; output is a pure ±500 correction (no offset).
8. **Motor mixing** — X-configuration in `MotorMixer::mix()`, invoked from `FlightController::updateControlLoop()`: `FL = throttle − roll + pitch`, `FR = throttle + roll + pitch`, `BL = throttle − roll − pitch`, `BR = throttle + roll − pitch`. Depends only on motor position, not physical spin direction (fixed mechanically by ESC wiring) — see README.md "Drehrichtungen" for the CW/CCW mapping and prop-pitch safety note.
9. **Status log** every 500 ms via `FlightController::logStatus()`, toggled at runtime with `l`.

### Module Map

| Path | Responsibility |
|------|---------------|
| [include/config.h](include/config.h) | All tunable parameters: ESC limits, PID defaults, flight parameters, `NORMALBETRIEB`/`TEST_KEYBOARD` switch |
| [include/pins.h](include/pins.h) | Single source of truth for all GPIO assignments |
| [include/myLogger.h](include/myLogger.h) | `LOG`/`LOG_FMT` macros — dual output to USB Serial + BT UART. Implemented in `src/myLogger.cpp` (routes through `CommChannel`) for the main firmware; each standalone tool under `test/` provides its own minimal `dlog()` writing straight to `Serial`, so it doesn't need to link `CommChannel`/`PIDController`/`Settings` |
| [lib/MotorMixer/MotorMixer.cpp](lib/MotorMixer/MotorMixer.cpp) | PWM to ESCs using native RP2040 `hardware/pwm.h` SDK (50 Hz, 20000 wrap, 1000–2000 µs) |
| [src/control/PIDController.cpp](src/control/PIDController.cpp) | Custom PID — `useOffset=true` (height) adds `THROTTLE_OFFSET_US` so output is absolute throttle clamped to `[ESC_MIN_US, ESC_MAX_US]`; `useOffset=false` (roll/pitch) outputs a pure ±500 correction; integral only accumulates while `enableIntegral(true)` |
| [lib/IMU/IMU.cpp](lib/IMU/IMU.cpp) | ICM-20948 9-DoF via the `wollewald/ICM20948_WE` library at I2C address **0x69** (board-specific quirk — datasheet implies 0x68 for AD0=GND); complementary filter (`alpha=0.98`) fusing gyro integration with accel-derived roll/pitch; I2C bus recovery via 9 SCL pulses in `IMU::begin(true)` before `Wire.begin()` |
| [lib/Barometer/Barometer.cpp](lib/Barometer/Barometer.cpp) | MS5611 (0x77); needs a 90 s warmup + calibration before it's trustworthy; ring-buffer filter; `BARO_TEMP_COEFF` compensates thermal drift; expects `Wire` already initialized by its caller |
| [lib/Ultrasonic/Ultrasonic.cpp](lib/Ultrasonic/Ultrasonic.cpp) | HC-SR04 on pins 8/6; valid range ~2–300 cm; preferred altitude source over barometer whenever `isValid()` |
| [lib/Battery/Battery.cpp](lib/Battery/Battery.cpp) | ADC pin 26, voltage divider; warns/critical via buzzer pin 10 |
| [src/comm/CommChannel.cpp](src/comm/CommChannel.cpp) | Transport-agnostic key/command parser and PID-tuning command processor. Constructor takes any `Stream&` (used for BT UART `Serial1` and USB `Serial` today; a TCP `Client` would work too, since it also derives from `Stream`). ANSI escape sequences are always discarded; `+`/`-` act immediately; other single-char commands (`A D H R L S ?`) resolve after a 200 ms timeout or on newline (whichever comes first); multi-char strings (`RP=`/`RI=`/`RD=`, `PP=`/`PI=`/`PD=`, height `P=`/`I=`/`D=`, `SAVE`, `RESET`) terminate on newline only. |
| [src/storage/Settings.cpp](src/storage/Settings.cpp) | EEPROM persistence for height PID Kp/Ki/Kd, validity marker byte |
| [src/control/FlightController.cpp](src/control/FlightController.cpp) | Owns flight state (`armed`, `targetHeightCm`, status-log/arm-pending timers), the three `PIDController` instances, and `MotorMixer`; provides `requestArm()`/`disarm()`/`recalibrate()`/`adjustTargetHeight()`/`toggleStatusLog()`, the safety check (`checkSafety()`), the PID+mixing loop (`updateControlLoop()`) and the status log (`logStatus()`) — the flight-control logic that used to live directly in `main.cpp::loop()` |
| [src/control/InputHandler.cpp](src/control/InputHandler.cpp) | Normal-mode key/command handling (ARM timeout, `switch(key)`) extracted out of `main.cpp::loop()`; translates key events into calls on `FlightController` |
| [src/testmode/TestModes.cpp](src/testmode/TestModes.cpp) | Only the `TEST_KEYBOARD` in-firmware test mode now (the other six moved to standalone tools under `test/`, see below) |

`MotorMixer`, `IMU`, `Barometer`, `Ultrasonic`, `Battery` live in `lib/<Name>/` (PlatformIO private libraries) rather than `src/`/`include/`, specifically so the standalone test tools under `test/` can link each one individually without pulling in `main.cpp` or unrelated modules — PlatformIO auto-links `lib/` into every environment and every `pio test` build regardless of `src_dir`/`test_build_src`. `PIDController`, `FlightController`, `CommChannel`, `Settings`, `InputHandler` stay in `src/`/`include/` since only the main firmware (and `TEST_KEYBOARD`) needs them. `main.cpp` itself now only declares the top-level sensor/comm objects (`baro`, `battery`, `ultrasonic`, `imu`, `comm`, `settings`) plus the single `flightController` instance, and wires them together in `setup()`/`loop()`.

### Test Modes

Six of the former seven `TEST_*` modes are now standalone tools under [test/](test/) — each its own tiny program (own `setup()`/`loop()`), built in isolation via PlatformIO's Unit Testing mechanism (`test_build_src` defaults to `false`, so `main.cpp`/the rest of `src/` is **not** compiled for these; `lib/` is still auto-linked, giving each tool just the driver module(s) it actually `#include`s):

```bash
pio test -e rpipico -f <name> --without-testing
pio device monitor
```

(`--without-testing` skips PlatformIO's Unity pass/fail parsing over Serial — these are interactive bench tools, not assertion-based tests. Equivalently, use the flask/test-tube icon in the PlatformIO IDE sidebar and pick the tool by name.)

(the `test_` prefix on each folder is required — PlatformIO only recognizes subdirectories named that way as individual test suites, otherwise it falls back to one catch-all test spanning the whole `test/` tree)

- `test/test_motors/` — all four motors together, plus an ESC-calibration sub-sequence (`c`/`k`/`m`); reads commands from `BT_UART` (`Serial1`) directly, same as before
- `test/test_motors_single/` — drive one motor by index (`1`=FL, `2`=FR, `3`=BR, `4`=BL); also via `BT_UART`
- `test/test_barometer/` — continuous pressure/altitude/temperature print
- `test/test_imu/` — continuous roll/pitch/AccZ print
- `test/test_ultrasonic/` — HC-SR04 distance print every 200 ms
- `test/test_i2c_scan/` — scans the I2C bus every 5 s (expects `0x69` IMU, `0x77` baro); includes an SDA-stuck-low hardware-fault check before scanning

`TEST_KEYBOARD` (BT/keyboard command echo + barometer print) is the **only** remaining `config.h`-gated in-firmware test mode — defined in [include/config.h](include/config.h), gated by the `NORMALBETRIEB` master switch (see Build System above): comment out `NORMALBETRIEB` and uncomment `TEST_KEYBOARD` to run it. It stayed in `main.cpp`/`TestModes.cpp` rather than becoming a standalone tool because it exercises `CommChannel::processCommand()` against real `PIDController`/`Settings` instances — essentially the whole input/tuning stack, not an isolable hardware driver.

`COMM_USE_BLUETOOTH` (also in [include/config.h](include/config.h)) is a separate, independent switch: it selects which `Stream` the single `comm` channel wraps (`Serial1`/BT UART when defined, `Serial`/USB when commented out) — not a test mode, and orthogonal to `NORMALBETRIEB`/`TEST_KEYBOARD` above, and irrelevant to the standalone `test/` tools (which don't use `comm`/`CommChannel` at all).

### Key Design Decisions

- **ICM-20948 IMU via `ICM20948_WE` library** — replaced a previous custom MPU9250 I2C driver, driven by repeated ESD failures of MPU9250 boards (see README "Sicherheit & Handhabung"). On this specific board the sensor answers at I2C address 0x69 even with AD0 tied to GND, not the datasheet's 0x68 — hardcoded in `IMU.h`; don't "fix" it back to 0x68.
- **Native RP2040 PWM SDK** (`hardware/pwm.h`) instead of `RP2040_PWM` library — more stable, no library dependency.
- **Custom PID** instead of FastPID — FastPID's coefficient clamping conflicted with required ranges.
- **Ultrasonic preferred over barometer** when in range (2–300 cm) — better accuracy and no warmup requirement.
- **I2C bus recovery** — sends 9 clock pulses to release a stuck SDA line before every `Wire.begin()`, both in `IMU::begin(true)` (normal operation) and in the standalone `test/test_i2c_scan/` tool (own copy, since that tool doesn't link `IMU`).
- **Integral anti-windup gated on liftoff** (`LIFTOFF_HEIGHT_CM`) — height/roll/pitch integrators stay at zero until the ultrasonic confirms the craft is airborne, and are cleared again on landing.
- **Single active input/log channel via a `comm` pointer, chosen at compile time** — `comm` is the *only* named `CommChannel*` in the code (no separate `bt`/`serial` globals). Which `Stream` it wraps is decided by the `COMM_USE_BLUETOOTH` switch in `config.h`: `comm = new CommChannel(Serial1)` (BT UART) when defined, `comm = new CommChannel(Serial)` (USB) otherwise — a single `#ifdef` in `setup()`, evaluated only after both `Serial` and `Serial1` (pins + `begin()`) are fully configured, since `TEST_MOTORS`/`TEST_MOTORS_SINGLE` read `BT_UART` directly regardless of the switch. There is no BT-primary/Serial-fallback redundancy; switching channels means flipping `COMM_USE_BLUETOOTH` and recompiling. `CommChannel` itself (parameterized by `Stream&`) uses one unified parsing scheme regardless of channel: ANSI arrow-key escape sequences are always discarded (harmless no-op for BT), single-char commands resolve via a 200 ms timeout so BT apps that don't send Enter still work, and `+`/`-` act instantly. Note: `d` (disarm) is not byte-instant — it resolves within ≤200 ms — because a byte-instant `d`/`D` would break the `D=<value>` (height Kd) tuning command the instant the `D` byte is read, before `=<value>` arrives.
- **Yaw = 0 currently** — gyro-based yaw stabilization deferred to Phase 3; only the complementary-filtered roll/pitch are used for attitude control.

### Planned Phases (not yet implemented)

- **Phase 2**: NRF24L01 remote control (library already in `platformio.ini`, SPI pins defined in `pins.h`, not yet wired into `main.cpp`)
- **Phase 3**: Yaw stabilization using ICM-20948 magnetometer + full gyro integration
- **Phase 4**: GPS-assisted position hold
- **Phase 5**: Autonomous flight routes

### Hardware Notes

See README.md for full hardware detail (pinout table, motor spin-direction verification, ESD handling procedure, power-on/off sequencing). Highlights relevant to code changes:

- **MS5611 not found**: check PS/NCS pins on the CJMCU-10DOF-style board are tied to 3.3 V; run `pio test -e rpipico -f test_i2c_scan --without-testing`.
- **Barometer drift indoors**: needs the full 90 s warmup and a recalibration (`r`) immediately before arming.
- **Arrow keys not recognized over USB**: use `pio device monitor`, not the VS Code built-in Serial Monitor — and remember BT is the intended primary input anyway.
- **Pico not detected by picotool**: hold BOOTSEL, flash `flash_nuke.uf2`, then re-flash normally.
- **ICM-20948 not found despite correct wiring**: confirm it enumerates at 0x69, not 0x68, via `pio test -e rpipico -f test_i2c_scan --without-testing`.
- **Motor spin direction vs. propeller pitch**: the CW/CCW assignment in README is specific to this physical board's ESC wiring, not a universal rule — always verify per-motor with `pio test -e rpipico -f test_motors_single --without-testing` (props off) before mounting propellers, and match propeller pitch (normal vs. pusher) to the observed direction.

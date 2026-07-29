# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a **PlatformIO** project targeting the **Raspberry Pi Pico** (RP2040) with the Earle Philhower Arduino core.

```bash
# Build
pio run

# Upload to device
pio run --target upload

# Monitor serial output (COM11 @ 115200 baud, see platformio.ini)
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor

# Build/flash one standalone hardware test tool (see Test Modes below) — its own
# [env:test_*], e.g. the IMU tool under src/tools/test_imu/ — main.cpp is NOT compiled
pio run -e test_imu --target upload
pio device monitor
```

There is no automated unit-test suite — validation happens via the hardware test tools below plus manual bench/flight testing (see README.md "Ein-/Ausschalten" for the arm/disarm power-on procedure). There is one repeatable interface-level check: the `serial-cli-test` skill drives the CLI over COM11 with a Python/pyserial script (`.claude/skills/serial-cli-test/height_test.py`) to verify `setHeight`/`getHeight` clamping — it never arms and only exercises `FlightController::setTargetHeightCm()`. It needs the CLI on USB (`CLI_USE_BLUETOOTH` commented out) — which is the current default.

Build flags in `platformio.ini`: `-DGLOBAL_DEBUG` is always on. The firmware has a **single mode** — normal flight operation ([src/mode/NormalMode.cpp](src/mode/NormalMode.cpp)); there is no longer a `NORMALBETRIEB`/`TEST_KEYBOARD` compile switch. Six former `TEST_*` modes are standalone tools under `src/tools/`, each its own PlatformIO environment (`[env:test_*]`), built via `pio run -e <name>` (see Test Modes below) and never touching `main.cpp`; the former keyboard/CLI test has no standalone tool — that input/tuning path is only reachable through the real firmware now.

Logging is controlled by two flags in `config.h`, consumed by `dlog()` in [src/myLogger.cpp](src/myLogger.cpp): `_SERIAL_LOG` (USB `Serial`) and `_BT_LOG` (`BT_UART`/`Serial1`). `dlog()` writes to the raw streams, deliberately independent of which stream the CLI shell is attached to — the two are configured separately and can be pointed at different channels. Use `LOG(msg)` and `LOG_FMT(fmt, ...)`.

**Current default: CLI on USB, logs on BT.** `CLI_USE_BLUETOOTH` is commented out and only `_BT_LOG` is set, so `pio device monitor` on COM11 gives a clean shell with no log traffic interleaved into what you type, while `[CTRL]`/`[SAFETY]` messages stream to the HC-06. The trade-off: autonomous events (safety disarm, battery warnings) are **not** visible on USB — enable `_SERIAL_LOG` as well if you want them mirrored there.

**No new libraries without checking with the user first** (project rule from README.md).

## Architecture Overview

The firmware implements a **cascaded PID altitude + attitude stabilizer** for a quadcopter running on RP2040. [src/main.cpp](src/main.cpp) is deliberately trivial — it includes only `<Arduino.h>` and [mode/NormalMode.h](include/mode/NormalMode.h), and its `setup()`/`loop()` just call `NormalMode::setup()`/`NormalMode::loop()`. [src/mode/NormalMode.cpp](src/mode/NormalMode.cpp) is the composition root: it *defines* the shared firmware objects as file-scope globals (`baro`, `battery`, `ultrasonic`, `flightController`, `settings`, `imu` — the same ones `cli.cpp` reaches via `extern`), sets up the CLI + sensors in `setup()`, and runs the control loop in `loop()`. The actual flight state and control logic (arm/disarm, safety checks, PID + motor mixing, status log) lives in [src/control/FlightController.cpp](src/control/FlightController.cpp), called from `NormalMode`. All hardware test tools are separate standalone programs under `src/tools/`, each its own `[env:test_*]` (see Test Modes below) — they never touch `main.cpp` or `NormalMode`.

### Control Loop (`NormalMode::loop()`)

1. **Sensor update** every loop iteration — barometer, ultrasonic, IMU all polled unconditionally in `NormalMode::loop()`.
2. **Safety checks** — `FlightController::checkSafety()` disarms (`FlightController::disarm()`) on IMU-not-ready or an altitude jump > 500 cm between consecutive loop iterations (based on barometer altitude).
3. **Input processing** — `cli::update()` ([src/comm/cli.cpp](src/comm/cli.cpp)) is the *only* input path; it runs first in `loop()`. The CLI wraps the `philj404/SimpleSerialShell` library and is bound to one stream, chosen by the `CLI_USE_BLUETOOTH` switch in `config.h` — `BT_UART`/`Serial1` when defined, `Serial`/USB otherwise (the current default). PID-tuning commands act on the `PIDController` instances owned by `FlightController` (`getPidHeight()`/`getPidRoll()`/`getPidPitch()`). See the CLI section below for the command set and its two non-obvious constraints.
4. **Arm timeout** — `FlightController::updateArmPendingTimeout()` expires a pending ARM confirmation after 3 s.
5. **Arm sequence** — `FlightController::requestArm()`: issuing `arm` twice within 3 s recalibrates the barometer, resets all three PID controllers, arms, and sets `targetHeightCm = 20`. `stop` (or a bare `d`) calls `FlightController::disarm()` (motors to `ESC_MIN_US`).
6. **Height PID** (every `PID_INTERVAL_MS` = 50 ms, in `FlightController::updateControlLoop()`) — error = `targetHeightCm − currentAltitude`; altitude source is ultrasonic when valid, barometer otherwise. Output includes the `THROTTLE_OFFSET_US` baseline.
7. **Anti-windup / liftoff gating** — the integral term on all three PID controllers only accumulates once `ultrasonic.isValid() && altitude > LIFTOFF_HEIGHT_CM` (`enableIntegral()`); it's cleared again on landing. This prevents integral windup while sitting on the ground pre-liftoff.
8. **Attitude PID** — roll/pitch corrections from IMU angles, target = 0°; output is a pure ±500 correction (no offset).
9. **Motor mixing** — X-configuration in `MotorMixer::mix()`, invoked from `FlightController::updateControlLoop()`: `FL = throttle − roll + pitch`, `FR = throttle + roll + pitch`, `BL = throttle − roll − pitch`, `BR = throttle + roll − pitch`. Depends only on motor position, not physical spin direction (fixed mechanically by ESC wiring) — see README.md "Drehrichtungen" for the CW/CCW mapping and prop-pitch safety note.
10. **Status log** every 500 ms via `FlightController::logStatus()`, toggled at runtime with `statusLog`.

### Module Map

| Path | Responsibility |
|------|---------------|
| [include/config.h](include/config.h) | All tunable parameters: ESC limits, PID defaults, flight parameters, `_SERIAL_LOG`/`_BT_LOG` log targets, `CLI_USE_BLUETOOTH` CLI-channel switch |
| [include/pins.h](include/pins.h) | Single source of truth for all GPIO assignments |
| [include/myLogger.h](include/myLogger.h) | `LOG`/`LOG_FMT` macros. Implemented in `src/myLogger.cpp`, which writes straight to `Serial` and/or `BT_UART` per `_SERIAL_LOG`/`_BT_LOG`; each standalone tool under `src/tools/` provides its own minimal `dlog()` writing only to `Serial`, so it doesn't need the rest of `src/` |
| [lib/MotorMixer/MotorMixer.cpp](lib/MotorMixer/MotorMixer.cpp) | PWM to ESCs using native RP2040 `hardware/pwm.h` SDK (50 Hz, 20000 wrap, 1000–2000 µs) |
| [src/control/PIDController.cpp](src/control/PIDController.cpp) | Custom PID — `useOffset=true` (height) adds `THROTTLE_OFFSET_US` so output is absolute throttle clamped to `[ESC_MIN_US, ESC_MAX_US]`; `useOffset=false` (roll/pitch) outputs a pure ±500 correction; integral only accumulates while `enableIntegral(true)` |
| [lib/IMU/IMU.cpp](lib/IMU/IMU.cpp) | ICM-20948 9-DoF via the `wollewald/ICM20948_WE` library at I2C address **0x69** (board-specific quirk — datasheet implies 0x68 for AD0=GND); complementary filter (`alpha=0.98`) fusing gyro integration with accel-derived roll/pitch; I2C bus recovery via 9 SCL pulses in `IMU::begin(true)` before `Wire.begin()` |
| [lib/Barometer/Barometer.cpp](lib/Barometer/Barometer.cpp) | MS5611 (0x77); needs a 90 s warmup + calibration before it's trustworthy; ring-buffer filter; `BARO_TEMP_COEFF` compensates thermal drift; expects `Wire` already initialized by its caller |
| [lib/Ultrasonic/Ultrasonic.cpp](lib/Ultrasonic/Ultrasonic.cpp) | HC-SR04 on pins 8/6; valid range ~2–300 cm; preferred altitude source over barometer whenever `isValid()` |
| [lib/Battery/Battery.cpp](lib/Battery/Battery.cpp) | ADC pin 26, voltage divider; warns/critical via buzzer pin 10 |
| [src/comm/cli.cpp](src/comm/cli.cpp) | The firmware's sole input path — a `SimpleSerialShell`-based CLI bound to `Serial1` or `Serial` per `CLI_USE_BLUETOOTH`. Naming: verbs for actions (`arm`, `stop`, `recalibrate`, `save`, `reset`, `statusLog`), `setX`/`getX` for values (`setHeight`, `getHeight`, `getArmed`), and one option-parsing command (`pid`). See the CLI section below for the `d` naming constraint. |
| [src/storage/Settings.cpp](src/storage/Settings.cpp) | EEPROM persistence for height PID Kp/Ki/Kd, validity marker byte |
| [src/control/FlightController.cpp](src/control/FlightController.cpp) | Owns flight state (`armed`, `targetHeightCm`, status-log/arm-pending timers), the three `PIDController` instances, and `MotorMixer`; provides `requestArm()`/`disarm()`/`recalibrate()`/`adjustTargetHeight()`/`toggleStatusLog()`, the safety check (`checkSafety()`), the PID+mixing loop (`updateControlLoop()`) and the status log (`logStatus()`) — the flight-control logic that used to live directly in `main.cpp::loop()` |
| [src/mode/NormalMode.cpp](src/mode/NormalMode.cpp) | Firmware composition root / sole entry point: defines the shared globals, does CLI + sensor init in `setup()`, runs the control loop in `loop()` (`cli::update()`, sensor updates, `FlightController::checkSafety()`/`updateArmPendingTimeout()`/`updateControlLoop()`/`logStatus()`). `main.cpp` just forwards to it |

`MotorMixer`, `IMU`, `Barometer`, `Ultrasonic`, `Battery` live in `lib/<Name>/` (PlatformIO private libraries) rather than `src/`/`include/`, specifically so the standalone tools under `src/tools/` can link each one individually without pulling in `main.cpp` or unrelated modules — PlatformIO auto-links `lib/` into every environment regardless of the `build_src_filter` in effect. `PIDController`, `FlightController`, `cli`, `Settings`, `NormalMode` stay directly under `src/`/`include/` since only the main firmware needs them, and the firmware env (`[env:rpipico]`) excludes `src/tools/` via `build_src_filter = +<*> -<tools/>`. `main.cpp` is a two-line shim (`NormalMode::setup()`/`loop()`); the shared globals and all wiring live in `NormalMode.cpp`.

### CLI (`src/comm/cli.cpp`)

The CLI is the firmware's only input path. It replaced `CommChannel`/`InputHandler`/`KeyEvent`, which were deleted once it covered their full command set — there is no longer a single-keypress parser with a 200 ms resolution timeout.

| Command | Notes |
|---|---|
| `arm` / `stop` | `arm` twice within 3 s to arm; `stop` disarms |
| `recalibrate` / `statusLog` | recalibrate only while disarmed |
| `save` / `reset` | height PID to/from EEPROM |
| `pid [axis] [-Kp/-Ki/-Kd v]` | read/write all PID coefficients — see below |
| `setHeight <cm>` / `getHeight` | clamped to `[THROTTLE_MIN_CM, MAX_HEIGHT_CM]` |
| `getArmed` | flight state |
| `help` | built into the library; lists everything |

**`pid`** replaced nine `setK*Height`/`Roll`/`Pitch` setters plus `getPid` with one command. `pid -h` prints its own option help.

```
pid                              → {"height":{...},"roll":{...},"pitch":{...}}
pid -height                      → {"height":{"Kp":2.0000,"Ki":0.0000,"Kd":0.0000}}
pid -height -Kd 10               set one coefficient
pid -height -Kd 10.1 -Ki 1 -Kp 0.1   several at once
pid -height -Kp 2 -roll -Kp 1    several axes in one call
```

The axis is *stateful*: it applies to every following `-K…` option, which is what allows more than one controller per call. There is deliberately **no default axis** — a `-Kp` before any axis flag is an error rather than silently landing in the height controller. Options compare case-insensitively and accept a German decimal comma. Output is always the axes that were named (all three when none were), printed *after* the writes, so the JSON reflects the value `PIDController::setKp()` actually stored — including its clamp to `[PID_COEFF_MIN, PID_COEFF_MAX]` = `[0, 255]`. Note `SimpleSerialShell` caps a line at 10 tokens, so at most three `-K…` pairs plus two axis flags fit in one call.

Four things about this module are load-bearing and easy to break:

- **No command name may start with `d`.** `cli::update()` pulls `d` out of the stream as a byte-instant emergency disarm *before* the shell sees it, so a command named `disarm` would fire on its first byte and leave `isarm` in the buffer — the same collision the old `CommChannel` had between `d` and `D=<value>`. Hence `stop`.
- **`d`/`+`/`-` are only intercepted at the start of a line**, tracked via the `atLineStart` flag. Mid-line they belong to an argument — otherwise `setHeight -10` would lose its minus sign. A 5 s idle timeout calls `shell.resetBuffer()` so an abandoned partial line can't leave the emergency stop disarmed.
- **Command feedback goes through `shell.print*()`**, so it always returns on the channel the command arrived on. `LOG()` is separate and goes wherever `_SERIAL_LOG`/`_BT_LOG` point — the two are deliberately decoupled.
- The shell is a **singleton with a single `attach()` stream** (USB *or* BT, never both — hence the `CLI_USE_BLUETOOTH` switch), matches command names **case-insensitively** (`strncasecmp`), and terminates a line on `\r` **or `;`** — the latter exists specifically for BT/BLE apps that can't send Enter, which is what makes running the CLI over `Serial1` practical.

`cli::begin()` prints a short greeting on whichever stream it attached to, so the channel never looks dead just because `LOG()` was pointed elsewhere.

### Test Modes

Six former `TEST_*` modes are standalone tools under [src/tools/](src/tools/) — each its own tiny program (own `setup()`/`loop()`) and its own PlatformIO environment `[env:test_<name>]`. Each env sets `build_src_filter = -<*> +<tools/test_<name>/>`, so **only that one tool folder** compiles (main.cpp/NormalMode/the rest of `src/` are excluded); `lib/` is still auto-linked, giving each tool just the driver module(s) it actually `#include`s. Each tool also provides its own minimal `dlog()` writing straight to `Serial` (so it doesn't link `cli`/`PIDController`/`Settings`):

```bash
pio run -e <name> --target upload
pio device monitor
```

(Equivalently, use the PlatformIO IDE sidebar → Project Tasks → the `test_<name>` env → Upload. A bare `pio run` builds only the firmware, thanks to `default_envs = rpipico`.)

- `src/tools/test_motors/` — all four motors together, plus an ESC-calibration sub-sequence (`c`/`k`/`m`); reads commands from `BT_UART` (`Serial1`) directly
- `src/tools/test_motors_single/` — drive one motor by index (`1`=FL, `2`=FR, `3`=BR, `4`=BL); also via `BT_UART`
- `src/tools/test_barometer/` — continuous pressure/altitude/temperature print
- `src/tools/test_imu/` — continuous roll/pitch/AccZ print
- `src/tools/test_ultrasonic/` — HC-SR04 distance print every 200 ms
- `src/tools/test_i2c_scan/` — scans the I2C bus every 5 s (expects `0x69` IMU, `0x77` baro); includes an SDA-stuck-low hardware-fault check before scanning

The former in-firmware `TEST_KEYBOARD` (BT/keyboard command echo + PID tuning) has **no** standalone tool: it exercised the input/tuning stack against real `PIDController`/`Settings` instances rather than an isolable hardware driver, so it can't be reproduced as a tool that excludes the rest of `src/`. That path is only reachable through the real firmware now (`help` in the CLI).

`CLI_USE_BLUETOOTH` in [include/config.h](include/config.h) (currently commented out) selects which `Stream` the CLI shell attaches to: `BT_UART`/`Serial1` when defined, `Serial`/USB otherwise. None of the `src/tools/` tools use `cli`.

### Key Design Decisions

- **ICM-20948 IMU via `ICM20948_WE` library** — replaced a previous custom MPU9250 I2C driver, driven by repeated ESD failures of MPU9250 boards (see README "Sicherheit & Handhabung"). On this specific board the sensor answers at I2C address 0x69 even with AD0 tied to GND, not the datasheet's 0x68 — hardcoded in `IMU.h`; don't "fix" it back to 0x68.
- **Native RP2040 PWM SDK** (`hardware/pwm.h`) instead of `RP2040_PWM` library — more stable, no library dependency.
- **Custom PID** instead of FastPID — FastPID's coefficient clamping conflicted with required ranges.
- **Ultrasonic preferred over barometer** when in range (2–300 cm) — better accuracy and no warmup requirement.
- **I2C bus recovery** — sends 9 clock pulses to release a stuck SDA line before every `Wire.begin()`, both in `IMU::begin(true)` (normal operation) and in the standalone `src/tools/test_i2c_scan/` tool (own copy, since that tool doesn't link `IMU`).
- **Integral anti-windup gated on liftoff** (`LIFTOFF_HEIGHT_CM`) — height/roll/pitch integrators stay at zero until the ultrasonic confirms the craft is airborne, and are cleared again on landing.
- **One CLI on one stream, chosen at compile time** — the shell is a singleton and can only serve a single `Stream`, so `CLI_USE_BLUETOOTH` in `config.h` picks it: `cli::begin(BT_UART)` when defined, `cli::begin(Serial)` otherwise — a single `#ifdef` in `NormalMode::setup()`, evaluated only after the chosen `Stream` (pins + `begin()`) is fully configured. There is no BT-primary/USB-fallback redundancy; switching channels means flipping the switch and recompiling. Logging is decoupled from this and is *not* a singleton: `dlog()` writes to `Serial` and/or `Serial1` per `_SERIAL_LOG`/`_BT_LOG`, so logs can go to both channels at once, or to the channel the shell is *not* on — which is the current setup (shell on USB, logs on BT).
- **Emergency disarm bypasses the shell** — `cli::update()` reads `d` (and `+`/`-`) straight off the stream before `shell.executeIfInput()`, so disarm needs one keypress instead of a full line plus Enter. The cost is a naming constraint (no command may start with `d`) and line-position tracking (`atLineStart`), both documented in the CLI section. The old `CommChannel` solved the same problem with a 200 ms single-char timeout, which is why its `d` was *not* byte-instant.
- **Yaw = 0 currently** — gyro-based yaw stabilization deferred to Phase 3; only the complementary-filtered roll/pitch are used for attitude control.

### Planned Phases (not yet implemented)

- **Phase 2**: NRF24L01 remote control (library already in `platformio.ini`, SPI pins defined in `pins.h`, not yet wired into `main.cpp`)
- **Phase 3**: Yaw stabilization using ICM-20948 magnetometer + full gyro integration
- **Phase 4**: GPS-assisted position hold
- **Phase 5**: Autonomous flight routes

### Hardware Notes

See README.md for full hardware detail (pinout table, motor spin-direction verification, ESD handling procedure, power-on/off sequencing). Highlights relevant to code changes:

- **MS5611 not found**: check PS/NCS pins on the CJMCU-10DOF-style board are tied to 3.3 V; run `pio run -e test_i2c_scan --target upload`.
- **Barometer drift indoors**: needs the full 90 s warmup and a `recalibrate` immediately before arming.
- **No `[CTRL]`/`[SAFETY]` messages over USB**: expected — `_SERIAL_LOG` is off, logs go to BT only. Define `_SERIAL_LOG` in `config.h` to mirror them onto USB. Conversely, if the CLI prompt is missing on COM11, check that `CLI_USE_BLUETOOTH` is still commented out.
- **Pico not detected by picotool**: hold BOOTSEL, flash `flash_nuke.uf2`, then re-flash normally.
- **ICM-20948 not found despite correct wiring**: confirm it enumerates at 0x69, not 0x68, via `pio run -e test_i2c_scan --target upload`.
- **Motor spin direction vs. propeller pitch**: the CW/CCW assignment in README is specific to this physical board's ESC wiring, not a universal rule — always verify per-motor with `pio run -e test_motors_single --target upload` (props off) before mounting propellers, and match propeller pitch (normal vs. pusher) to the observed direction.

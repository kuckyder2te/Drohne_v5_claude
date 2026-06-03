# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a **PlatformIO** project targeting the **Raspberry Pi Pico** (RP2040) with the Earle Philhower Arduino core.

```bash
# Build
pio run

# Upload to device
pio run --target upload

# Monitor serial output (COM8 @ 115200 baud)
pio device monitor

# Build + upload + monitor in one step
pio run --target upload && pio device monitor
```

Build flags in `platformio.ini`: `-DGLOBAL_DEBUG` is always on. Test modes are enabled by uncommenting defines in [include/config.h](include/config.h).

## Architecture Overview

The firmware implements a **cascaded PID altitude + attitude stabilizer** for a quadcopter running on RP2040.

### Control Loop (normal operation, `src/main.cpp`)

The main loop runs continuously at ~20 Hz for the PID step:

1. **Sensor update** — barometer, ultrasonic, IMU all polled every loop iteration
2. **Input processing** — USB Serial + Bluetooth UART (arrow keys, arm/disarm, PID tuning)
3. **Safety checks** — disarms on IMU failure or altitude jump > 500 cm
4. **Height PID** (50 ms interval) — error = `targetHeightCm - currentAltitude`; altitude source is ultrasonic when valid, barometer otherwise
5. **Attitude PID** — roll/pitch corrections computed from IMU angles (target = 0°)
6. **Motor mixing** — X-configuration: `FL = throttle - roll + pitch`, `FR = throttle + roll + pitch`, `BL = throttle - roll - pitch`, `BR = throttle + roll - pitch`
7. **Status log** every 500 ms

### Module Map

| Path | Responsibility |
|------|---------------|
| [include/config.h](include/config.h) | All tunable parameters: ESC limits, PID defaults, test mode switches, filter sizes |
| [include/pins.h](include/pins.h) | Single source of truth for all GPIO assignments |
| [src/control/MotorMixer](src/control/MotorMixer.cpp) | PWM to ESCs using native RP2040 `hardware/pwm.h` SDK (50 Hz, 20000 wrap, 1000–2000 µs) |
| [src/control/PIDController](src/control/PIDController.cpp) | Custom PID — height mode adds `ESC_OFFSET` so output is absolute throttle; attitude mode outputs pure correction |
| [src/sensor/IMU](src/sensor/IMU.cpp) | MPU9250 via I2C; computes roll/pitch from atan2(accel); auto-recovers a hung I2C bus |
| [src/sensor/Barometer](src/sensor/Barometer.cpp) | MS5607 (MS5611-compatible); 30 s warmup + 10 s thermal stabilization + 20-sample calibration at arm |
| [src/sensor/Ultrasonic](src/sensor/Ultrasonic.cpp) | Dual HC-SR04; 5-sample moving average; preferred altitude source over barometer |
| [src/sensor/Battery](src/sensor/Battery.cpp) | ADC pin 26, 100 k/20 k divider (×6), warns at 10.5 V / cuts at 10.0 V |
| [src/comm/KeyboardInput](src/comm/KeyboardInput.cpp) | ANSI escape parser for USB Serial + BT UART; feeds command chars to main loop |
| [src/comm/BluetoothConfig](src/comm/BluetoothConfig.cpp) | Real-time PID tuning via BT (`P=`, `I=`, `D=`, `RP=`, `RI=`, `RD=`, `S`, `R`) |
| [src/storage/Settings](src/storage/Settings.cpp) | EEPROM persistence for PID coefficients (16 bytes, validated by marker 0xAB) |

### Test Modes

Defined in [include/config.h](include/config.h). Uncomment exactly one to run that test; leave all commented for normal flight operation:

- `TEST_MOTORS` — all four motors sequentially
- `TEST_MOTORS_SINGLE` — individual motor index test
- `TEST_BAROMETER` — continuous pressure/altitude print
- `TEST_KEYBOARD` — echo all received key codes
- `TEST_I2C_SCAN` — scan I2C bus and print found addresses
- `TEST_IMU` — continuous roll/pitch/yaw print
- `TEST_ULTRASONIC` — dual HC-SR04 distance readings

### Key Design Decisions

- **Native RP2040 PWM SDK** (`hardware/pwm.h`) instead of `RP2040_PWM` library — more stable, no library dependency
- **Custom PID** instead of FastPID — FastPID's coefficient clamping conflicted with required ranges
- **Ultrasonic preferred over barometer** when in range (2–300 cm) — ±0.5 cm vs ±1 cm accuracy
- **I2C bus recovery** in IMU — sends 9 clock pulses to release a stuck SDA line
- **Barometer temperature coefficient** (`BARO_TEMP_COEFF` in config.h) — compensates for thermal drift per °C

### Planned Phases (not yet implemented)

- **Phase 2**: NRF24L01 remote control (library already in `platformio.ini`, SPI pins defined in `pins.h`)
- **Phase 3**: Yaw stabilization using MPU9250 magnetometer

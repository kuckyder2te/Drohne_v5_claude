# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a **PlatformIO** project targeting the **Raspberry Pi Pico 2 W** (RP2350, Cortex-M33 @ 150 MHz, 520 kB RAM, 4 MB Flash) with the Earle Philhower Arduino core. The firmware environment is `[env:rpipico2w]` (`default_envs`), board `rpipico2w`.

It ran on a **Pico (RP2040)** before, and nothing in the code is RP2350-specific — the pinout is identical and the drivers are unchanged. Two things carried the port and matter if anyone ever switches back or forward:

- **The PWM clock divider must come from the actual system clock.** `pwm_init_pin()` in [lib/MotorMixer/MotorMixer.cpp](lib/MotorMixer/MotorMixer.cpp) computes `clock_get_hz(clk_sys) / 1e6` so the PWM counter runs at exactly 1 MHz — one count = 1 µs, `wrap` 20000 = 50 Hz — on any clock. The old hardcoded `125.0f` was only right for the RP2040's 125 MHz; at the RP2350's 150 MHz it would have produced a 60 Hz frame with 0.833 µs per count, making every ESC pulse ~17 % short (`ESC_MIN_US` 1000 would have looked like 833 µs to the ESC).
- **The cross-core synchronisation in [include/control/SharedState.h](include/control/SharedState.h) is unchanged and stays that way.** It uses only aligned 32-bit `volatile` accesses plus mutexes — correct on both chips. The Cortex-M33 does have real LDREX/STREX, so `__atomic_*` *would* work now, but converting buys nothing and would break back-portability.

USB VID/PID differ from the RP2040 Pico, so **the COM port can change** — `upload_port`/`monitor_port` in `platformio.ini` are pinned to COM11 and may need adjusting after the swap.

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

Logging goes through the `bakercp/Logger` library, wrapped by [include/myLogger.h](include/myLogger.h). Use `LOGGER_NOTICE(msg)` and `LOGGER_NOTICE_FMT(fmt, ...)` (also `…_VERBOSE`, `…_WARNING`, `…_ERROR`, `…_FATAL`). Four things about this wrapper are easy to trip over:

- **`myLogger.h` must include `<Logger.h>` before its own `#define`s**, and nothing may include `<Logger.h>` directly. `LOGGER_NOTICE` is a macro here, while `Logger` declares methods of its own; if the library header were parsed after the macros were defined, the macro would expand inside the class declaration.
- **The library's default log level is `WARNING`, so `NOTICE` output is suppressed until someone raises it.** `NormalMode::setup()` calls `Logger::setLogLevel(Logger::_DEBUG_)` (`_DEBUG_` comes from `-D_DEBUG_=NOTICE` in `platformio.ini`); each tool under `src/tools/` calls `Logger::setLogLevel(Logger::NOTICE)` itself. Forget it and the build succeeds but stays silent.
- **The `*_FMT` macros `sprintf` into one shared `logBuf`** declared in `myLogger.h` and defined in `src/myLogger.cpp` — plus once per tool, since `build_src_filter` excludes `myLogger.cpp` from tool builds while `lib/` (which uses the macros) is linked everywhere. All definitions must keep the same size as the declaration. `sprintf` is unbounded: the longest line in the project is `FlightController::logStatus()` at ~106 chars, which is why the buffer is 160. A longer format string needs the buffer grown with it.
- **Every line carries `__PRETTY_FUNCTION__` as its module prefix**, so a status line reads `[NOTICE]:void FlightController::logStatus(const Battery&, …):[CTRL] Ziel: …` — around 200 characters. On the 9600-baud BT link that is a real cost.

`localLogger()` in [src/myLogger.cpp](src/myLogger.cpp) is registered via `Logger::setOutputFunction()` and writes to the raw streams per two flags in `config.h`: `_SERIAL_LOG` (USB `Serial`) and `_BT_LOG` (`BT_UART`/`Serial1`), both allowed at once. This is deliberately independent of which stream the CLI shell is attached to.

`platformio.ini` sets **`lib_ldf_mode = deep+`** because of this: the modules under `lib/` include `myLogger.h`, which now pulls in `<Logger.h>`. The default `chain` finder does not follow includes from the project's `include/` into a third-party library, so building `lib/Barometer` etc. fails with `Logger.h: No such file or directory`. For the same reason, anything under `lib/` that needs a `config.h` constant must include it explicitly — `config.h` used to arrive transitively through the old `myLogger.h`, which no longer includes it.

**Current default: CLI on USB, logs on BT.** `CLI_USE_BLUETOOTH` is commented out and only `_BT_LOG` is set, so `pio device monitor` on COM11 gives a clean shell with no log traffic interleaved into what you type, while `[CTRL]`/`[SAFETY]` messages stream to the HC-06. The trade-off: autonomous events (safety disarm, battery warnings) are **not** visible on USB — enable `_SERIAL_LOG` as well if you want them mirrored there.

**No new libraries without checking with the user first** (project rule from README.md).

## Architecture Overview

The firmware implements a **cascaded PID altitude + attitude stabilizer** for a quadcopter running on RP2040. [src/main.cpp](src/main.cpp) is deliberately trivial — it includes only `<Arduino.h>` and [mode/NormalMode.h](include/mode/NormalMode.h), and its `setup()`/`loop()` just call `NormalMode::setup()`/`NormalMode::loop()`. [src/mode/NormalMode.cpp](src/mode/NormalMode.cpp) is the composition root: it *defines* the shared firmware objects as file-scope globals (`baro`, `battery`, `ultrasonic`, `flightController`, `settings`, `imu` — the same ones `cli.cpp` reaches via `extern`), sets up the CLI + sensors in `setup()`, and runs the control loop in `loop()`. The actual flight state and control logic (arm/disarm, safety checks, PID + motor mixing, status log) lives in [src/control/FlightController.cpp](src/control/FlightController.cpp), called from `NormalMode`. All hardware test tools are separate standalone programs under `src/tools/`, each its own `[env:test_*]` (see Test Modes below) — they never touch `main.cpp` or `NormalMode`.

### Dual-Core-Aufteilung (seit dem Autotune-Umbau)

Der Regelkreis läuft auf **beiden Kernen**. Auslöser war eine Messung: `NormalMode::loop()` lief real mit **~8–10 Hz**, nicht mit den nominellen 20 Hz, weil `Barometer::update()` 70 ms und `Ultrasonic::update()` 31–55 ms blockierend `delay()`en. `PID_INTERVAL_MS = 50` war damit wirkungslos, und bei ~150 ms Gesamttotzeit (plus IMU-DLPF_6 mit 5,7 Hz Bandbreite) gab es *keinen* Satz PID-Werte, der stabil schweben konnte — Tuning wäre sinnlos gewesen.

| Kern | Aufgabe | Takt |
|---|---|---|
| **Kern 1** ([src/control/AttitudeLoop.cpp](src/control/AttitudeLoop.cpp)) | IMU lesen, Roll/Pitch-PID, Motor-Mixing, Recorder, Relay-Autotune | `ATTITUDE_RATE_HZ` = 400 Hz |
| **Kern 0** ([src/mode/NormalMode.cpp](src/mode/NormalMode.cpp)) | Ultraschall, Batterie, CLI, Höhen-PID, Logging, Watchdog | ~50–100 Hz |

`setup1()`/`loop1()` sind schwache Symbole im earlephilhower-Core; sind sie definiert, startet `main()` Kern 1 automatisch — kein Build-Flag nötig. Kern 1 läuft **vor** `setup()` von Kern 0 an, deshalb wartet `setup1()` auf `shared::g_core0Ready`.

**Der Barometer ist abgeschaltet** (`BARO_ENABLED` in `config.h`, auskommentiert). MS5611 und ICM-20948 hängen am selben `Wire`-Bus; statt einen Mutex um jede Transaktion zu legen, bekommt Kern 1 den I2C-Bus exklusiv. Höhenquelle ist damit ausschließlich der Ultraschall. Wiedereinschalten heißt: Mutex um **alle** Wire-Transaktionen in `Barometer` *und* `IMU` — und der Mutex muss `requestFrom()` samt aller folgenden `read()` umfassen, `TwoWire::_buff` ist gemeinsamer Zustand.

Fünf Dinge sind hier tragend und leicht zu zerstören:

- **Kein `LOGGER_*` auf Kern 1.** Die `*_FMT`-Makros schreiben in den *einen* globalen `logBuf` aus `src/myLogger.cpp`; eine Ausgabe von Kern 1 zerstört eine gleichzeitig laufende von Kern 0 mitten im `sprintf`. Deshalb hat `PIDController` ein `setQuiet(bool)`, das `AttitudeLoop::begin()` auf seinen Instanzen setzt — `setKp`/`reset`/`enableIntegral` loggen sonst. Meldungen laufen stattdessen über `CoreTlm::abortCode`, den Kern 0 in `abortText()` übersetzt.
- **Synchronisiert wird nur über ausgerichtete 32-Bit-`volatile`-Zugriffe (auf dem Bus atomar) und Mutexe, nicht über `__atomic_*`.** Der Entwurf stammt vom RP2040: Cortex-M0+ hat kein LDREX/STREX, GCC löst die Builtins dort über eine Interrupt-Sperre auf, die nur gegen ISRs desselben Kerns schützt, **nicht gegen den anderen Kern**. Auf dem RP2350 (Cortex-M33) gäbe es echte Exclusive-Zugriffe, aber der vorhandene Ansatz ist auf beiden Chips korrekt und bleibt so. Beides in [include/control/SharedState.h](include/control/SharedState.h): `CoreCmd` per Mutex + Generationszähler, `CoreTlm` per Seqlock, `g_estop`/`g_beatC0`/`g_beatC1` lock-frei.
- **Kern 1 wartet nie auf Kern 0.** `cmdFetch()` prüft lock-frei den Generationszähler und holt die neue Fassung nur, wenn `mutex_try_enter()` sofort gelingt — sonst regelt es mit der alten Kopie weiter. Ein CLI-Kommando kann den Regeltakt so nicht stören.
- **Der Not-Aus verlässt sich nicht auf Kern 1.** `cli::update()` setzt bei `d` erst `g_estop`, dann schreibt Kern 0 die PWM-Register **selbst** über `MotorMixer::stopFast()` (logfrei, reine Registerzugriffe, von jedem Kern sicher). Kern 1 prüft `g_estop` als allererste Anweisung in `step()`.
- **Beidseitiger Herzschlag.** Steht Kern 1, laufen die Motoren sonst mit dem letzten Wert weiter — der gefährlichste Zustand des Umbaus. Kern 0 prüft `g_beatC1` gegen `ATTITUDE_WATCHDOG_MS` (150 ms) und disarmt; Kern 1 prüft `g_beatC0` gegen `CORE0_WATCHDOG_MS` (500 ms) und stoppt die Motoren. `stats -hang` blockiert Kern 1 gezielt für 1 s, um das nachzuweisen — **dieser Test muss bestanden sein, bevor ein Propeller montiert wird.**

Ein Nebeneffekt, der bewusst so ist: **`pid -save` und `tune -save` sind im armierten Zustand gesperrt.** `EEPROM.commit()` ruft `rp2040.idleOtherCore()` und friert Kern 1 für die gesamte Flash-Löschung ein — die Motoren stünden solange auf ihrem letzten Wert.

### Control Loop (`NormalMode::loop()`)

**Kern 0** (`NormalMode::loop()`), ~50–100 Hz:

1. **Input processing** — `cli::update()` ([src/comm/cli.cpp](src/comm/cli.cpp)) is the *only* input path; it runs first in `loop()`. The CLI wraps the `philj404/SimpleSerialShell` library and is bound to one stream, chosen by the `CLI_USE_BLUETOOTH` switch in `config.h` — `BT_UART`/`Serial1` when defined, `Serial`/USB otherwise (the current default). PID-tuning commands act on the `PIDController` instances owned by `FlightController` (`getPidHeight()`/`getPidRoll()`/`getPidPitch()`). See the CLI section below for the command set and its non-obvious constraints.
2. **Sensor update auf festen Kadenzen** — Ultraschall alle `ULTRA_UPDATE_MS` (50 ms), Barometer alle `BARO_UPDATE_MS` (200 ms, aber nur mit `BARO_ENABLED`), Batterie ungetaktet. Vorher lief beides in *jedem* Durchlauf, was den Kern auf ~9 Hz drückte. Der IMU wird hier gar nicht mehr angefasst — er gehört Kern 1.
3. **Telemetrie holen** — `shared::tlmRead()` liefert Lage, Drehraten, Motorwerte und Loop-Rate von Kern 1.
4. **Kern-1-Watchdog** — bleibt `g_beatC1` länger als `ATTITUDE_WATCHDOG_MS` stehen, setzt Kern 0 `g_estop`, ruft `stopFast()` und disarmt.
5. **Safety checks** — `FlightController::checkSafety()` disarmt bei IMU-not-ready (aus `CoreTlm::imuReady`) oder einem Höhensprung > 500 cm. Im Prüfstands-/Tune-Modus ist der Höhensprung-Check abgeschaltet — auf der Wippe misst der Ultraschall je nach Neigung Unsinn.
6. **Arm timeout** — `FlightController::updateArmPendingTimeout()` expires a pending ARM confirmation after 3 s.
7. **Arm sequence** — `FlightController::requestArm()`: `arm` zweimal innerhalb 3 s setzt alle Regler zurück, armt, setzt `targetHeightCm = 20` und `mode = MODE_FLIGHT`. Die Barometer-Rekalibrierung entfällt, solange `BARO_ENABLED` aus ist. `stop` (oder ein blankes `d`) ruft `disarm()`.
8. **Height PID** (alle `PID_INTERVAL_MS` = 50 ms, in `FlightController::updateControlLoop()`) — error = `targetHeightCm − currentAltitude`, Quelle ist der Ultraschall. Das Ergebnis geht als `throttleUs` per `publish()` an Kern 1; **hier wird nicht mehr gemischt.**
9. **Status log** alle 500 ms via `FlightController::logStatus()`, zur Laufzeit mit `statusLog` umschaltbar. Zeigt jetzt zusätzlich Roll/Pitch und die Ist-Loop-Rate von Kern 1.

**Kern 1** (`attitude::step()`), 400 Hz:

1. **Not-Aus zuerst** — `g_estop` wird als allererste Anweisung geprüft.
2. **Takt halten** — selbstkorrigierend auf `ATTITUDE_PERIOD_US`, ohne Drift; Überschreitungen werden als `overruns` gezählt statt aufgeholt.
3. **Vorgaben holen** — `cmdFetch()`, nie blockierend (s. o.).
4. **Kern-0-Watchdog** — `g_beatC0` gegen `CORE0_WATCHDOG_MS`.
5. **IMU lesen** — `imu.update(nowUs)` mit vorgegebenem Zeitstempel.
6. **Attitude PID** — `computeWithRate(target, angle, gyroRate, dt)`. Der D-Anteil kommt aus der **Gyro-Rate**, nicht aus `(error − lastError)/dt`: bei 400 Hz ist der Differenzenquotient auf einem verrauschten Winkel unbrauchbar, und die Rate *ist* die Ableitung des Winkels. Nebeneffekt: kein Derivative-Kick bei Sollwertsprüngen — die erzeugt das Relais beim Autotuning garantiert.
7. **Motor mixing** — X-Konfiguration in `MotorMixer::mix()`: `FL = throttle − roll + pitch`, `FR = throttle + roll + pitch`, `BL = throttle − roll − pitch`, `BR = throttle + roll − pitch`, geklemmt auf `[ESC_MIN_US, _maxUs]`. Hängt nur von der Motorposition ab, nicht von der Drehrichtung (mechanisch über die ESC-Verkabelung festgelegt) — siehe README.md "Drehrichtungen".
8. **Recorder/Autotune** — in `MODE_BENCH`/`MODE_TUNE`, siehe unten.
9. **Telemetrie veröffentlichen** + Herzschlag.

**Anti-windup / liftoff gating** liegt weiterhin bei `ultrasonic.isValid() && altitude > LIFTOFF_HEIGHT_CM`; Kern 0 ermittelt das Gate und schickt es als `CoreCmd::integralOn` mit. Am Prüfstand greift es nie — dort steht das Gerät fest.

### Module Map

| Path | Responsibility |
|------|---------------|
| [include/config.h](include/config.h) | All tunable parameters: ESC limits, PID defaults, flight parameters, `_SERIAL_LOG`/`_BT_LOG` log targets, `CLI_USE_BLUETOOTH` CLI-channel switch |
| [include/pins.h](include/pins.h) | Single source of truth for all GPIO assignments |
| [include/myLogger.h](include/myLogger.h) | `LOGGER_*` macros over the `bakercp/Logger` library, plus the shared `logBuf` used by the `*_FMT` variants. Output function `localLogger()` lives in `src/myLogger.cpp` and writes to `Serial` and/or `BT_UART` per `_SERIAL_LOG`/`_BT_LOG`; each standalone tool under `src/tools/` defines its own `logBuf` and sets its own log level, since `myLogger.cpp` isn't in its build |
| [lib/MotorMixer/MotorMixer.cpp](lib/MotorMixer/MotorMixer.cpp) | PWM to ESCs using the native Pico-SDK `hardware/pwm.h` (50 Hz, 20000 wrap, 1000–2000 µs). Der Teiler kommt aus `clock_get_hz(clk_sys)`, damit der Zähler unabhängig vom Systemtakt exakt 1 MHz läuft — siehe Build System. Schaltet außerdem die ESC-Stromversorgung über `PIN_ESC_POWER` (`powerOn()`/`powerOff()`/`isPowered()`, `begin(bool autoPower)`) — siehe ESC-Stromversorgung unten |
| [src/control/PIDController.cpp](src/control/PIDController.cpp) | Custom PID — `useOffset=true` (height) adds `THROTTLE_OFFSET_US` so output is absolute throttle clamped to `[ESC_MIN_US, ESC_MAX_US]`; `useOffset=false` (roll/pitch) outputs a pure ±500 correction; integral only accumulates while `enableIntegral(true)`. Zeitbasis ist **`micros()`** (bei 2,5 ms Zyklus wäre `millis()` bis zu 40 % daneben). `computeWithRate()` nimmt den D-Anteil aus einer gemessenen Rate; `setQuiet()` schaltet alle `LOGGER_*` ab — Pflicht für jede Instanz auf Kern 1 |
| [src/control/SharedState.cpp](src/control/SharedState.cpp) | Datenaustausch zwischen den Kernen: `CoreCmd` (Kern 0 → 1, Mutex + Generationszähler), `CoreTlm` (Kern 1 → 0, Seqlock), `TuneResult`, sowie `g_estop`/`g_beatC0`/`g_beatC1` lock-frei |
| [src/control/AttitudeLoop.cpp](src/control/AttitudeLoop.cpp) | Der Regelkreis auf Kern 1: IMU, Roll/Pitch-PID, Mixing, Betriebsarten `MODE_FLIGHT`/`MODE_BENCH`/`MODE_TUNE`, Abbruchüberwachung. `axisProject()`/`axisDrive()`/`axisMotors()` bilden die vier Prüfstandsachsen (2 Flugachsen + 2 Motordiagonalen) auf die IMU-Winkel bzw. die Mixer-Eingänge ab — der Mixer selbst kennt nur Roll und Pitch. Loggt nie und wartet nie auf Kern 0 |
| [src/control/Recorder.cpp](src/control/Recorder.cpp) | Ringpuffer für den Messschrieb: 20 B/Sample × `REC_CAPACITY` (3000) = 60 kB in der `.bss`, bei `REC_DECIMATION` = 2 sind das 15 s @ 200 Hz. Ein Erzeuger (Kern 1), ein Verbraucher (Kern 0), lockfrei — der Verbraucher liest nur nach `isActive() == false` |
| [src/control/RelayTuner.cpp](src/control/RelayTuner.cpp) | Relay-Feedback-Autotune: Zweipunktregler mit Hysterese, Periodenmessung über die steigenden Flanken, Ku/Tu-Statistik. Läuft auf Kern 1, weil die Umschaltzeitpunkte µs-Auflösung brauchen |
| [lib/IMU/IMU.cpp](lib/IMU/IMU.cpp) | ICM-20948 9-DoF via the `wollewald/ICM20948_WE` library at I2C address **0x69** (board-specific quirk — datasheet implies 0x68 for AD0=GND); complementary filter fusing gyro integration with accel-derived roll/pitch; I2C bus recovery via 9 SCL pulses in `IMU::begin(true)` before `Wire.begin()`. **DLPF: Gyro `DLPF_3` (51,2 Hz), Accel `DLPF_4` (23,9 Hz)** — vorher `DLPF_6` (5,7 Hz) auf beiden, was >30 ms Gruppenlaufzeit bedeutete. `alpha` wird je Zyklus als `tau/(tau+dt)` berechnet (`IMU_TAU_S`) statt fest verdrahtet; Gyro-Bereich 500 dps wegen der Relais-Anregung |
| [lib/Barometer/Barometer.cpp](lib/Barometer/Barometer.cpp) | MS5611 (0x77); needs a 90 s warmup + calibration before it's trustworthy; ring-buffer filter; `BARO_TEMP_COEFF` compensates thermal drift; expects `Wire` already initialized by its caller |
| [lib/Ultrasonic/Ultrasonic.cpp](lib/Ultrasonic/Ultrasonic.cpp) | HC-SR04 on pins 8/6; valid range ~2–300 cm; preferred altitude source over barometer whenever `isValid()` |
| [lib/Battery/Battery.cpp](lib/Battery/Battery.cpp) | ADC pin 26, voltage divider; warns/critical via buzzer pin 10 |
| [src/comm/cli.cpp](src/comm/cli.cpp) | The firmware's sole input path — a `SimpleSerialShell`-based CLI bound to `Serial1` or `Serial` per `CLI_USE_BLUETOOTH`. Naming: verbs for actions (`arm`, `stop`, `recalibrate`, `save`, `reset`, `statusLog`), `setX`/`getX` for values (`setHeight`, `getHeight`, `getArmed`), and one option-parsing command (`pid`). See the CLI section below for the `d` naming constraint. |
| [src/storage/Settings.cpp](src/storage/Settings.cpp) | EEPROM persistence for all three PID controllers (`PidCoeffs` × height/roll/pitch, 12 B each) plus one validity-marker byte. The marker doubles as a layout version — bumped to `0xAC` when roll/pitch were added, so a pre-existing EEPROM falls back to defaults instead of being misread. Addresses live only in `Settings.h` (they used to be duplicated in `config.h`) |
| [src/control/FlightController.cpp](src/control/FlightController.cpp) | Owns flight state (`armed`, `targetHeightCm`, status-log/arm-pending timers), the three `PIDController` instances, and `MotorMixer`; provides `requestArm()`/`disarm()`/`recalibrate()`/`adjustTargetHeight()`/`toggleStatusLog()`, the safety check (`checkSafety()`), the PID+mixing loop (`updateControlLoop()`) and the status log (`logStatus()`) — the flight-control logic that used to live directly in `main.cpp::loop()` |
| [src/mode/NormalMode.cpp](src/mode/NormalMode.cpp) | Firmware composition root / sole entry point: defines the shared globals, does CLI + sensor init in `setup()`, runs the control loop in `loop()` (`cli::update()`, sensor updates, `FlightController::checkSafety()`/`updateArmPendingTimeout()`/`updateControlLoop()`/`logStatus()`). `main.cpp` just forwards to it |

`MotorMixer`, `IMU`, `Barometer`, `Ultrasonic`, `Battery` live in `lib/<Name>/` (PlatformIO private libraries) rather than `src/`/`include/`, specifically so the standalone tools under `src/tools/` can link each one individually without pulling in `main.cpp` or unrelated modules — PlatformIO auto-links `lib/` into every environment regardless of the `build_src_filter` in effect. `PIDController`, `FlightController`, `cli`, `Settings`, `NormalMode` stay directly under `src/`/`include/` since only the main firmware needs them, and the firmware env (`[env:rpipico2w]`) excludes `src/tools/` via `build_src_filter = +<*> -<tools/>`. `main.cpp` is a two-line shim (`NormalMode::setup()`/`loop()`); the shared globals and all wiring live in `NormalMode.cpp`.

### CLI (`src/comm/cli.cpp`)

The CLI is the firmware's only input path. It replaced `CommChannel`/`InputHandler`/`KeyEvent`, which were deleted once it covered their full command set — there is no longer a single-keypress parser with a 200 ms resolution timeout.

| Command | Notes |
|---|---|
| `arm` / `stop` | `arm` twice within 3 s to arm; `stop` disarms |
| `recalibrate` / `statusLog` | recalibrate only while disarmed |
| `pid [axis] [-Kp/-Ki/-Kd v] [-save] [-reset]` | read/write/persist all PID coefficients — see below |
| `setHeight <cm>` / `getHeight` | clamped to `[THROTTLE_MIN_CM, MAX_HEIGHT_CM]` |
| `getArmed` | flight state |
| `stats [-hang]` | Zustand von Kern 1 als JSON; `-hang` ist der Watchdog-Nachweis |
| `bench [...]` | Prüfstandsbetrieb auf einer Achse (`-roll`/`-pitch`/`-flbr`/`-frbl`) + Messschrieb — siehe unten |
| `tune [...]` | Relay-Feedback-Autotune, gleiche Achswahl — siehe unten |
| `help` | command list + the immediate-key chapter — see below |

**`pid`** replaced nine `setK*Height`/`Roll`/`Pitch` setters plus `getPid`, `save` and `reset` with one command. `pid -h` prints its own option help.

```
pid                              → {"height":{...},"roll":{...},"pitch":{...}}
pid -height                      → {"height":{"Kp":2.0000,"Ki":0.0000,"Kd":0.0000}}
pid -height -Kd 10               set one coefficient
pid -height -Kd 10.1 -Ki 1 -Kp 0.1   several at once
pid -height -Kp 2 -roll -Kp 1    several axes in one call
pid -save                        all three controllers → EEPROM
pid -reset                       all three → config.h defaults, clear EEPROM
pid -height -Kp 2 -save          tune and persist in one line
```

The axis is *stateful*: it applies to every following `-K…` option, which is what allows more than one controller per call. There is deliberately **no default axis** — a `-Kp` before any axis flag is an error rather than silently landing in the height controller. Options compare case-insensitively and accept a German decimal comma.

`-save`/`-reset` always cover **all three** controllers (the EEPROM block has a single validity marker, so a partial save isn't representable) and run in a fixed order regardless of where they appear in the line: **reset → coefficient writes → save**. That two-pass design is what makes `pid -save -height -Kp 2` persist the *new* value rather than the pre-change one. Because they are global, they also force the JSON output to show all three axes — a reply narrowed to one axis would misrepresent what was written.

Output is printed *after* the writes, so the JSON reflects what `PIDController::setKp()` actually stored — including its clamp to `[PID_COEFF_MIN, PID_COEFF_MAX]` = `[0, 255]`. Note `SimpleSerialShell` caps a line at 10 tokens, so at most three `-K…` pairs plus two axis flags fit in one call.

**`pid` is atomic.** It runs in three passes — collect `-reset`/`-save` flags, then *validate the whole line and queue the writes*, then apply. Nothing is touched until the line parses cleanly, so `pid -reset -height -Kp 2 -Ki abc` changes nothing at all rather than leaving you with a reset controller and a half-applied tuning step. Keep new options inside this structure: validation belongs in pass 2, side effects in pass 3.

**Numeric arguments are validated**, by `parseFloatDe()` in `cli.cpp` — used by both `pid` and `setHeight`. It exists because bare `strtof()` returns `0.0f` for `"abc"`, so a typo like `-Kp o.5` would silently zero a coefficient. It rejects trailing garbage (`12abc`, `1.2.3`), bare signs, `nan`/`inf`, overflow, and over-long input, while accepting a German decimal comma, exponents, and leading `+`/`-`.

Four things about this module are load-bearing and easy to break:

- **No command name may start with `d`.** `cli::update()` pulls `d` out of the stream as a byte-instant emergency disarm *before* the shell sees it, so a command named `disarm` would fire on its first byte and leave `isarm` in the buffer — the same collision the old `CommChannel` had between `d` and `D=<value>`. Hence `stop`.
- **`d`/`+`/`-` are only intercepted at the start of a line**, tracked via the `atLineStart` flag. Mid-line they belong to an argument — otherwise `setHeight -10` would lose its minus sign. A 5 s idle timeout calls `shell.resetBuffer()` so an abandoned partial line can't leave the emergency stop disarmed.
- **`help` is shadowed, not replaced.** `cmdHelp` re-registers the name `help`; `addCommand()` inserts an equal name *ahead* of the existing entry and `execute()` takes the first match, so the custom one wins. It calls the library's `SimpleSerialShell::printHelp()` for the generated command list and appends the immediate-key chapter — those keys can never appear in the generated list because they aren't shell commands. Known cosmetic wart: the listing shows **two** `help` lines, the custom one and the library's built-in. `SimpleSerialShell` has no `removeCommand()` and `firstCommand` is private, so the stale entry can't be hidden without patching a managed dependency.
- **Command feedback goes through `shell.print*()`**, so it always returns on the channel the command arrived on. `LOGGER_NOTICE()` is separate and goes wherever `_SERIAL_LOG`/`_BT_LOG` point — the two are deliberately decoupled.
- The shell is a **singleton with a single `attach()` stream** (USB *or* BT, never both — hence the `CLI_USE_BLUETOOTH` switch), matches command names **case-insensitively** (`strncasecmp`), and terminates a line on `\r` **or `;`** — the latter exists specifically for BT/BLE apps that can't send Enter, which is what makes running the CLI over `Serial1` practical.

`cli::begin()` prints a short greeting on whichever stream it attached to, so the channel never looks dead just because `LOGGER_NOTICE()` was pointed elsewhere.

### Prüfstand (`bench`) und Autotune (`tune`)

Beide arbeiten auf einer **1-Achsen-Wippe**: das Gerät ist so eingespannt, dass es nur um *eine* Achse kippen kann. Der Höhenregler ist dabei komplett abgeschaltet und die Throttle fest — es geht ausschließlich um die Lageregelung.

```
bench -roll -throttle 1300 -max 1600 -go     Pruefstand starten (2x -go bestaetigen)
bench -flbr -go                               dasselbe auf der Motordiagonalen FL/BR
bench -dump [-n 500]                          Messschrieb als CSV (nur disarmt)
tune -roll -h 60 -eps 1.0 -go                 Autotune starten
tune -flbr -h 60 -go                          Autotune auf der Motordiagonalen
tune -show                                    Ergebnis ansehen, ohne zu uebernehmen
tune -apply                                   Kp und Kd uebernehmen
tune -save                                    uebernehmen und ins EEPROM (nur disarmt)
```

#### Vier Achsen: zwei Flugachsen, zwei Motordiagonalen

`-roll`/`-pitch` sind die Flugachsen. `-flbr`/`-frbl` sind die **physischen Motorachsen** des X-Rahmens — gedacht für den Fall, dass sich die Drohne mechanisch leichter auf einer Motordiagonalen einspannen lässt als auf einer Flugachse. Benannt sind sie nach den beiden Motoren, die die Wippe *antreiben*; die anderen beiden liegen auf der Wippenstange und bleiben exakt auf der Basis-Throttle stehen:

| Option | Wippenstange auf | angetrieben von | Winkel |
|---|---|---|---|
| `-roll` | Längsachse | FL+BL gegen FR+BR | `roll` |
| `-pitch` | Querachse | FL+FR gegen BL+BR | `pitch` |
| `-flbr` | Diagonale FR–BL | FL gegen BR | `(pitch − roll)·0,7071` |
| `-frbl` | Diagonale FL–BR | FR gegen BL | `(pitch + roll)·0,7071` |

Drei Dinge daran sind tragend:

- **Der `MotorMixer` bleibt unverändert.** Eine Diagonalkorrektur `c` wird in `attitude::axisDrive()` als `rollOut = ∓c/2, pitchOut = +c/2` eingespeist; die vorhandene X-Mischung liefert daraus `FL = t+c`, `BR = t−c` und lässt `FR` und `BL` rechnerisch exakt auf `t`. Es gibt also keinen Sonderpfad im Mixer und keine zweite Mischformel, die auseinanderlaufen könnte. Die Halbierung ist kein Fudge-Faktor: sie hält `c` in derselben Einheit wie bei Roll/Pitch (Abweichung *eines* Motors in µs), damit `h` auf allen vier Achsen dasselbe bedeutet.
- **Der Faktor 0,7071 = 1/√2 zwischen Diagonale und Flugachse** (`DIAG_AXIS_GAIN` in `config.h`, Herleitung dort). Er kommt allein aus dem Hebelarm: auf der Diagonalen wirken nur 2 Motoren, dafür mit Hebel `a·√2` statt `a`, also `2√2` statt `4` Einheiten Moment. Das Trägheitsmoment ist beim symmetrischen X um beide Diagonalen *gleich* dem um Roll/Pitch (jeweils `4ma²`), fällt also heraus. **Bei gestrecktem Rahmen (Deadcat) gilt das nicht** — dann stimmen weder die 45°-Projektion noch die Trägheitsgleichheit.
- **Der Faktor wirkt in beide Richtungen, damit der Kreis sich schließt.** `tune -flbr` → `-apply` schreibt `Kp·0,7071` in **roll und pitch zugleich** (die Diagonale liegt symmetrisch zu beiden — das ist der eigentliche Gewinn: ein Lauf statt zwei). Umgekehrt fährt `bench -flbr` den Regler `s_pidDiag`, der die Roll-Beiwerte *durch* 0,7071 geteilt bekommt. Dadurch zeigt die Wippe dasselbe Regelverhalten wie später der Rollregler im Flug, ohne dass jemand von Hand umrechnet. Wer eine der beiden Richtungen entfernt, bricht den Rundlauf.

`tune -show` gibt für eine Diagonale beides aus: `Ku`/`Kp`/`Ki`/`Kd` gelten für die Diagonale, die zusätzlichen `KuAxis`/`KpAxis`/`KiAxis`/`KdAxis` sind die umgerechneten Werte für roll+pitch — und **nur diese** schreibt `-apply`.

**Wie `tune` misst.** Statt eines PID-Reglers wirkt ein Zweipunktregler mit Hysterese auf die Achse. Der treibt die Strecke von selbst in eine Dauerschwingung, und zwar genau bei der kritischen Frequenz, an der ein P-Regler an der Stabilitätsgrenze stünde. Der Vorteil gegenüber „Kp erhöhen bis es schwingt": die Amplitude bleibt durch `h` begrenzt und wächst nicht unkontrolliert. Aus der Schwingung folgen `Tu` (Periodendauer, aus den steigenden Flanken) und `Ku = 4h / (π·√(a²−ε²))`.

**Einheitenprobe:** `h` steht in µs (Mixer-Korrektur), `a` in Grad, also hat `Ku` die Einheit µs/Grad — genau die Einheit von `Kp` in diesem Code (`rollCorr = Kp · error[Grad]` geht direkt in `mix()`). `Ku` ist damit unmittelbar mit `Kp` vergleichbar.

**Einstellregeln** (`-rule`, Parallelform `out = Kp·e + Ki·∫e + Kd·ė`):

| Regel | Kp | Ki | Kd |
|---|---|---|---|
| `zn` (klassisch) | 0,60·Ku | 1,20·Ku/Tu | 0,075·Ku·Tu |
| `pi` | 0,45·Ku | 0,54·Ku/Tu | 0 |
| `pessen` | 0,70·Ku | 1,75·Ku/Tu | 0,105·Ku·Tu |
| `some` | 0,33·Ku | 0,66·Ku/Tu | 0,110·Ku·Tu |
| **`no` (Vorgabe)** | **0,20·Ku** | 0,40·Ku/Tu | 0,066·Ku·Tu |

Warum nicht Ziegler-Nichols als Vorgabe: ZN zielt auf ein Amplitudenverhältnis von 1:4 je Periode, also ~25 % Überschwingen und nur etwa Faktor 2 Verstärkungsreserve. Die Wippe bildet aber nicht alle Totzeiten des Flugs ab (ESC-Ansprechzeit, Propellerhochlauf, Rahmenelastizität), und das Trägheitsmoment ist ein anderes. Bei Reserve 2 genügt eine andere Batteriespannung, um in die Instabilität zu kippen. **Das Ergebnis ist ein Startwert, kein Endergebnis** — vor dem ersten freien Flug Kd halbieren und mit Ki = 0 beginnen.

Vier Entwurfsentscheidungen, die leicht falsch „korrigiert" werden:

- **`-apply` setzt nur Kp und Kd; Ki braucht das ausdrückliche `-ki`.** Bei realistischem Ku ≈ 20 µs/° und Tu ≈ 0,3 s ergäbe `no` ein Ki ≈ 27; das Integral akkumuliert Grad·Sekunden und ist auf ±500 begrenzt — nach 1 s bei 5° Fehler wäre der Regler in der Sättigung. Zudem hängt `_integralEnabled` am Liftoff-Gate, das auf der Wippe nie greift.
- **Ein berechneter Koeffizient außerhalb `[0, 255]` verweigert `-apply`**, statt sich von `_clampCoeff()` still auf 255 klemmen zu lassen. Der Clamp ist als Tippfehlerschutz gedacht, nicht als Ventil für eine entgleiste Rechnung.
- **Bei `tune` ist die Hilfe `-help`, nicht `-h`** — `-h` ist dort die Relaisamplitude und der weitaus häufiger getippte Parameter. `bench` und `pid` nehmen weiterhin `-h`.
- **`bench -go` und `tune -go` verlangen eine zweite Bestätigung innerhalb 3 s**, dasselbe Muster wie `requestArm()`. Ein Tippfehler soll keine Motoren anwerfen.

Abbrüche werden auf Kern 1 jeden Zyklus geprüft (Winkelgrenze in 3 Zyklen in Folge, Timeout, `g_estop`, Kern-0-Herzschlag, Disarm, IMU-Fehler, und bei `tune` zusätzlich „keine Umschaltung > `TUNE_NOSWITCH_MS`" = keine Schwingung). Der Code landet in `CoreTlm::abortCode`, Kern 0 übersetzt ihn in `abortText()` — Kern 1 darf nicht loggen.

**Reihenfolge der Inbetriebnahme** (die ersten beiden Schritte ohne Propeller). `<achse>` steht für die Achse, auf der die Wippe tatsächlich eingespannt ist — `-roll`, `-pitch`, `-flbr` oder `-frbl`; in **allen** Schritten dieselbe:

1. `stats` → `loopHz` muss ~400 sein, `overruns` = 0.
2. `stats -hang` → muss innerhalb 150 ms `[SAFETY] Kern 1 antwortet nicht` auslösen. **Ohne diesen bestandenen Test darf kein Propeller montiert werden.**
3. `tune <achse> -h 60 -go` ohne Propeller → muss nach 3 s mit `ABORT_NOSWITCH` enden (es kann keine Schwingung entstehen). Prüft den Abbruchpfad.
4. **Vorzeichenprobe der Diagonalprojektion, ohne Propeller** (entfällt bei `-roll`/`-pitch`). Gerät von Hand so kippen, dass **FL nach oben** geht, und dabei `stats` lesen. `roll` und `pitch` müssen sich in **entgegengesetzte** Richtungen bewegen — dann stimmt `-flbr`. Bewegen sie sich gleichsinnig, ist für diesen Rahmen `-frbl` die FL/BR-Achse und die beiden Optionen sind vertauscht. Das ist kein Schönheitsfehler: `axisProject()` und `axisDrive()` müssen dasselbe Vorzeichen haben, sonst wird aus der Gegen- eine **Mitkopplung** und die Achse läuft beim ersten `bench` weg. Danach zusätzlich `bench <achse> -throttle 1150 -max 1250 -go` fahren und im `bench -dump`-CSV prüfen, dass die beiden Motoren *auf* der Wippenstange unbewegt auf der Basis-Throttle stehen.
5. Mit Propellern, eingespannt: `arm`, `tune <achse> -go`, danach `bench -dump`. Im CSV muss `out_us` eine saubere Rechteckschwingung zwischen −h und +h zeigen und `angle_deg` eine gleichmäßige Schwingung. `spread` < 0,1.
6. **Validierung der Methode:** `pid -roll -Kp <Ku> -Ki 0 -Kd 0` setzen und `bench <achse> -go`. Per Definition von Ku muss die Achse jetzt *grenzstabil* schwingen — schwingt sie auf oder klingt sie ab, ist Ku falsch und alle abgeleiteten Werte sind wertlos. **Auf einer Diagonalen ist hier `KuAxis` einzutragen, nicht `Ku`** — `bench` rechnet die Roll-Beiwerte selbst wieder auf die Diagonale hoch, ein direkt eingetragenes `Ku` wäre um √2 zu groß.
7. Erst danach `tune -apply`.

Der Messschrieb geht über `shell.print` auf den CLI-Kanal, also aktuell USB @115200 (≈ 14 s für den vollen Puffer). Über BT @9600 wären es ~156 s — deshalb ist der Dump nur disarmiert erlaubt und zählt in der Druckschleife `g_beatC0` weiter, sonst liefe der Herzschlag ab.

### ESC-Stromversorgung (MOSFET an GP28)

Die vier ESCs hängen nicht direkt am LiPo, sondern hinter einem Logic-Level-N-FET (IRLZ44N, Low-Side) an **`PIN_ESC_POWER` = GP28** (Board-Pin 34). HIGH = ESCs am Strom. Grund: **ein ESC leitet seine Betriebsart aus dem Signal ab, das beim Hochlaufen anliegt** — MIN (1000 µs) = Normalbetrieb, MAX (2000 µs) = Kalibriermodus. Vorher bekam alles gleichzeitig Strom, und die Kalibrierung ging nur über physisches Ab- und Anstecken des LiPo (so stand es in `printMotorHelp()`).

`MotorMixer::begin(bool autoPower = true)` hält die Reihenfolge ein und ist der einzige Ort, an dem sie steht: Pin auf OUTPUT + LOW → PWM aufsetzen → `stop()` (MIN) → `ESC_PWM_SETTLE_MS` (100 ms, ≥ 2 Rahmen bei 50 Hz) → `powerOn()` → `ESC_BOOT_MS` (2000 ms ESC-Eigeninitialisierung). Beide Konstanten in `config.h`.

Vier Punkte, die leicht kaputtgehen:

- **`powerOn()` fasst die PWM absichtlich nicht an.** Was gerade ausgegeben wird, *ist* die Betriebsart, in der der ESC hochläuft — ein „Sicherheitshalber MIN" in `powerOn()` würde den Kalibrierpfad (`k`) unbrauchbar machen. Wer Normalbetrieb will, sorgt vorher selbst für MIN.
- **Der Not-Aus schaltet den Strom nicht ab.** Stromlose ESCs im Flug heißen freier Fall; richtig ist MIN auf allen vier Kanälen (`stopFast()`). Die Firmware schaltet nach `FlightController::begin()` einmal ein und lässt an — `powerOff()` gibt es nur in den Werkzeugen.
- **Hardware: Pulldown ~10 k zwischen Gate und Masse.** Vom Reset bis zum ersten `pinMode()` ist der GPIO hochohmig; ohne Pulldown hängt das Gate in der Luft und die ESCs können beim Reset, im BOOTSEL-Modus oder beim Flashen unkontrolliert Strom bekommen. Der Firmware-Pfad allein reicht dafür nicht.
- **Die Werkzeuge starten stromlos** (`motors.begin(false)`), die Firmware nicht. `test_motors`/`test_motors_single` laufen oft mit montierten Propellern; dort legt erst `p` (bzw. `k`) Spannung auf die ESCs.

Kalibrierung in `test_motors` ohne Steckerziehen: `c` (Strom aus) → `k` (MAX ausgeben, dann einschalten) → Piepstöne abwarten → `m` (MIN). `k` verweigert die Arbeit, wenn die ESCs schon am Strom sind — nachträgliches MAX bringt einen laufenden ESC nicht in die Kalibrierung.

### Test Modes

Six former `TEST_*` modes are standalone tools under [src/tools/](src/tools/) — each its own tiny program (own `setup()`/`loop()`) and its own PlatformIO environment `[env:test_<name>]`. Each env sets `build_src_filter = -<*> +<tools/test_<name>/>`, so **only that one tool folder** compiles (main.cpp/NormalMode/the rest of `src/` are excluded); `lib/` is still auto-linked, giving each tool just the driver module(s) it actually `#include`s. Each tool also defines its own `logBuf` and calls `Logger::setLogLevel(Logger::NOTICE)` in `setup()`, since `src/myLogger.cpp` isn't in its build while `lib/` — which uses the `*_FMT` macros — is linked anyway:

```bash
pio run -e <name> --target upload
pio device monitor
```

(Equivalently, use the PlatformIO IDE sidebar → Project Tasks → the `test_<name>` env → Upload. A bare `pio run` builds only the firmware, thanks to `default_envs = rpipico2w`.)

- `src/tools/test_motors/` — all four motors together, plus an ESC-calibration sub-sequence (`c`/`k`/`m`) und `p` als Stromschalter; startet stromlos, reads commands from `BT_UART` (`Serial1`) directly
- `src/tools/test_motors_single/` — drive one motor by index (`1`=FL, `2`=FR, `3`=BR, `4`=BL), `p` schaltet den ESC-Strom; startet stromlos; also via `BT_UART`
- `src/tools/test_barometer/` — continuous pressure/altitude/temperature print
- `src/tools/test_imu/` — continuous roll/pitch/AccZ print
- `src/tools/test_ultrasonic/` — HC-SR04 distance print every 200 ms
- `src/tools/test_i2c_scan/` — scans the I2C bus every 5 s (expects `0x69` IMU, `0x77` baro); includes an SDA-stuck-low hardware-fault check before scanning

The former in-firmware `TEST_KEYBOARD` (BT/keyboard command echo + PID tuning) has **no** standalone tool: it exercised the input/tuning stack against real `PIDController`/`Settings` instances rather than an isolable hardware driver, so it can't be reproduced as a tool that excludes the rest of `src/`. That path is only reachable through the real firmware now (`help` in the CLI).

`CLI_USE_BLUETOOTH` in [include/config.h](include/config.h) (currently commented out) selects which `Stream` the CLI shell attaches to: `BT_UART`/`Serial1` when defined, `Serial`/USB otherwise. None of the `src/tools/` tools use `cli`.

### Key Design Decisions

- **ICM-20948 IMU via `ICM20948_WE` library** — replaced a previous custom MPU9250 I2C driver, driven by repeated ESD failures of MPU9250 boards (see README "Sicherheit & Handhabung"). On this specific board the sensor answers at I2C address 0x69 even with AD0 tied to GND, not the datasheet's 0x68 — hardcoded in `IMU.h`; don't "fix" it back to 0x68.
- **Native Pico-SDK PWM** (`hardware/pwm.h`) instead of the `RP2040_PWM` library — more stable, no library dependency. Der Taktteiler wird aus `clock_get_hz(clk_sys)` berechnet statt fest verdrahtet, damit ein Zählschritt auf jedem Systemtakt 1 µs bleibt.
- **Custom PID** instead of FastPID — FastPID's coefficient clamping conflicted with required ranges.
- **Ultrasonic preferred over barometer** when in range (2–300 cm) — better accuracy and no warmup requirement.
- **I2C bus recovery** — sends 9 clock pulses to release a stuck SDA line before every `Wire.begin()`, both in `IMU::begin(true)` (normal operation) and in the standalone `src/tools/test_i2c_scan/` tool (own copy, since that tool doesn't link `IMU`).
- **Integral anti-windup gated on liftoff** (`LIFTOFF_HEIGHT_CM`) — height/roll/pitch integrators stay at zero until the ultrasonic confirms the craft is airborne, and are cleared again on landing.
- **One CLI on one stream, chosen at compile time** — the shell is a singleton and can only serve a single `Stream`, so `CLI_USE_BLUETOOTH` in `config.h` picks it: `cli::begin(BT_UART)` when defined, `cli::begin(Serial)` otherwise — a single `#ifdef` in `NormalMode::setup()`, evaluated only after the chosen `Stream` (pins + `begin()`) is fully configured. There is no BT-primary/USB-fallback redundancy; switching channels means flipping the switch and recompiling. Logging is decoupled from this and is *not* a singleton: `localLogger()` writes to `Serial` and/or `Serial1` per `_SERIAL_LOG`/`_BT_LOG`, so logs can go to both channels at once, or to the channel the shell is *not* on — which is the current setup (shell on USB, logs on BT).
- **Emergency disarm bypasses the shell** — `cli::update()` reads `d` (and `+`/`-`) straight off the stream before `shell.executeIfInput()`, so disarm needs one keypress instead of a full line plus Enter. The cost is a naming constraint (no command may start with `d`) and line-position tracking (`atLineStart`), both documented in the CLI section. The old `CommChannel` solved the same problem with a 200 ms single-char timeout, which is why its `d` was *not* byte-instant.
- **Yaw = 0 currently** — gyro-based yaw stabilization deferred to Phase 3; only the complementary-filtered roll/pitch are used for attitude control.
- **Lageregelung auf Kern 1 statt nicht-blockierender Sensortreiber** — die Alternative wäre gewesen, `Barometer::update()` und `Ultrasonic::update()` in State-Machines zu zerlegen. Der zweite Kern war ungenutzt und liefert das Ergebnis ohne Eingriff in erprobte Treiber; der Preis ist die Synchronisation (siehe Dual-Core-Abschnitt) und der abgeschaltete Barometer.
- **D-Anteil aus der Gyro-Rate** (`computeWithRate()`) statt aus `(error − lastError)/dt` — bei 400 Hz ist der Differenzenquotient auf einem verrauschten Winkel unbrauchbar, und die Rate ist die Ableitung ohnehin schon. Beseitigt zugleich den Derivative-Kick bei Sollwertsprüngen.
- **Relay-Feedback statt aufsteigendem Kp** für die Ermittlung von Ku/Tu — die Amplitude bleibt durch `h` begrenzt, statt beim Suchen der Stabilitätsgrenze unkontrolliert zu wachsen.

### Planned Phases (not yet implemented)

- **Phase 2**: NRF24L01 remote control (library already in `platformio.ini`, SPI pins defined in `pins.h`, not yet wired into `main.cpp`)
- **Phase 3**: Yaw stabilization using ICM-20948 magnetometer + full gyro integration
- **Phase 4**: GPS-assisted position hold
- **Phase 5**: Autonomous flight routes

### Hardware Notes

See README.md for full hardware detail (pinout table, motor spin-direction verification, ESD handling procedure, power-on/off sequencing). Highlights relevant to code changes:

- **Kein Barometer im Flugbetrieb**: erwartet — `BARO_ENABLED` ist in `config.h` auskommentiert, damit Kern 1 den I2C-Bus exklusiv hat. `recalibrate` meldet das entsprechend, Höhe kommt nur vom Ultraschall. Das Barometer selbst ist unverändert und über `pio run -e test_barometer` weiterhin prüfbar.
- **`loopHz` in `stats` deutlich unter 400**: `overruns` mitprüfen. Steigt der Wert, braucht ein Zyklus länger als `ATTITUDE_PERIOD_US` — meist ein versehentlich auf Kern 1 gelandeter blockierender Aufruf (`delay()`, `LOGGER_*`, `Serial.print`).
- **Motoren reagieren nicht, ESCs bleiben still**: ESC-Strom prüfen — GP28 muss HIGH sein. In den Werkzeugen erst `p` drücken (sie starten stromlos); in der Firmware macht das `MotorMixer::begin()`. Piept ein ESC dauerhaft statt zu armen, lag beim Einschalten nicht MIN an (Reihenfolge in `begin()` verändert?).
- **MS5611 not found**: check PS/NCS pins on the CJMCU-10DOF-style board are tied to 3.3 V; run `pio run -e test_i2c_scan --target upload`.
- **Barometer drift indoors**: needs the full 90 s warmup and a `recalibrate` immediately before arming (nur relevant mit `BARO_ENABLED`).
- **No `[CTRL]`/`[SAFETY]` messages over USB**: expected — `_SERIAL_LOG` is off, logs go to BT only. Define `_SERIAL_LOG` in `config.h` to mirror them onto USB. Conversely, if the CLI prompt is missing on COM11, check that `CLI_USE_BLUETOOTH` is still commented out.
- **Pico not detected by picotool**: hold BOOTSEL, flash `flash_nuke.uf2`, then re-flash normally.
- **ICM-20948 not found despite correct wiring**: confirm it enumerates at 0x69, not 0x68, via `pio run -e test_i2c_scan --target upload`.
- **Motor spin direction vs. propeller pitch**: the CW/CCW assignment in README is specific to this physical board's ESC wiring, not a universal rule — always verify per-motor with `pio run -e test_motors_single --target upload` (props off) before mounting propellers, and match propeller pitch (normal vs. pusher) to the observed direction.

### README.md Is Partially Stale

README.md predates the CLI/`FlightController`/`NormalMode` refactor described above and still documents an older architecture in several sections: the "Projektstruktur" tree (`control/MotorMixer.h`, `sensor/IMU.h`, `comm/BluetoothConfig.h`, `comm/KeyboardInput.h` — none of these paths exist anymore), the class diagram (`KeyboardInput`/`KeyEvent`/`BluetoothConfig`), and the "Bluetooth-Befehle" table (`P=x.x`/`I=x.x`/`D=x.x`/`S`/`R`/`?`, superseded by the `pid` CLI command). Trust this CLAUDE.md and the actual source over those sections. README's ESD-handling and power-on/off-sequencing sections are still current and are what the "Hardware Notes" above point to.

**The README pinout table itself is also stale and should not be trusted** — its "Pinbelegung" section (motor table around README.md line 70-75 and the code block around line 101-106) lists `PIN_MOTOR_FL=11, FR=12, BL=14, BR=13`, but the actual `include/pins.h` defines `PIN_MOTOR_FL=12, PIN_MOTOR_FR=13, PIN_MOTOR_BR=14, PIN_MOTOR_BL=15` — every motor pin number has shifted and BL/BR are swapped relative to what README shows. `include/pins.h` is the only source to read for actual GPIO assignments (README even says so in its own text right above the stale table, it just never got updated after a pin change). The CW/CCW spin-direction *mapping logic* and its caveat ("absolute CW/CCW assignment is not universal, only diagonal-pairs-match-and-adjacent-pairs-oppose is physically required") remain conceptually valid — only the concrete pin numbers in that table are wrong. This matters at the safety-relevant step of matching propeller pitch to a physically-verified spin direction (`test_motors_single`): identifying "FL" by the README's pin number would drive the wrong motor.

Two harmless leftovers from the same era: `include/sensor/` and `src/sensor/` are empty directories, and root-level `test_i2c_modules.cpp` is an orphaned file outside `src/` that PlatformIO never compiles — none of the three are part of the current build.

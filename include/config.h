#pragma once

// ── Debug Ausgabe ──────────────────────────────────────────
// Ziel der LOGGER_NOTICE()/LOGGER_NOTICE_FMT()-Ausgaben, beides gleichzeitig moeglich.
// Unabhaengig davon, wo die CLI-Shell haengt (siehe CLI_USE_BLUETOOTH).
//#define _SERIAL_LOG   // USB Serial - zuschalten, um Logs auch auf COM11 mitzulesen
#define _BT_LOG         // BT-UART (Serial1)

// ── Kanal der CLI-Shell ────────────────────────────────────
// Die Shell (SimpleSerialShell) ist ein Singleton mit genau EINEM Stream.
// Aktiv: CLI liegt auf Serial1 (BT-UART). Auskommentiert: CLI liegt auf
// Serial (USB) - dann funktioniert auch der serial-cli-test-Skill (COM11).
//#define CLI_USE_BLUETOOTH

// ── Betriebsmodus ────────────────────────────────────────────
// Die Firmware kennt nur noch den Normalbetrieb (Flugbetrieb, siehe
// src/mode/NormalMode.cpp). Alle Hardware-Testwerkzeuge sind eigenstaendige
// PlatformIO-Umgebungen unter src/tools/ (siehe src/tools/README, z.B.
// "pio run -e test_imu --target upload").

// Temperaturkompensation: Druckkorrektur pro °C Temperaturdifferenz
// Empirisch ermittelt für MS5607 Sensor
#define BARO_TEMP_COEFF 0.5f

// ESC PWM-Parameter (Standard: 1000–2000 µs)
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000
#define ESC_FREQ_HZ 50

// ── Bluetooth HC-06 (UART0) ────────────────────────────────
#define BT_UART Serial1
#define BT_BAUD 9600

// ── Flugparameter ──────────────────────────────────────────
#define TARGET_HEIGHT_CM 50
#define MAX_HEIGHT_CM 100
#define THROTTLE_STEP 50

// ── EEPROM-Adressen ────────────────────────────────────────
// Stehen jetzt ausschliesslich in include/storage/Settings.h - hier lagen
// sie frueher zusaetzlich, was beim Aendern still auseinanderlaufen konnte.

// ── PID Roll/Pitch ─────────────────────────────────────────
#define PID_KP_ROLL 0.5f // klein anfangen!
#define PID_KI_ROLL 0.0f
#define PID_KD_ROLL 0.0f
#define PID_KP_PITCH 0.5f
#define PID_KI_PITCH 0.0f
#define PID_KD_PITCH 0.0f

// ── Sollwerte Lage ─────────────────────────────────────────
#define TARGET_ROLL_DEG 0.0f
#define TARGET_PITCH_DEG 0.0f

// ── PID Startwerte ─────────────────────────────────────────
#define PID_KP_HEIGHT 2.0f
#define PID_KI_HEIGHT 0.0f
#define PID_KD_HEIGHT 0.0f

// ── Regelkreis ─────────────────────────────────────────────
#define PID_INTERVAL_MS 50      // 20 Hz Regelfrequenz
#define THROTTLE_MIN_CM 5       // Untergrenze Zielhöhe
#define THROTTLE_OFFSET_US 1500 // Basis-Throttle: Motoren laufen an

// ── Anti-Windup: Liftoff-Schwelle ──────────────────────────
// Integral-Term aktiv erst wenn Ultraschall > dieser Wert.
// Anpassen falls Sensor nicht auf Bodenniveau montiert ist.
#define LIFTOFF_HEIGHT_CM 6.0f  // cm über Boden = "abgehoben"
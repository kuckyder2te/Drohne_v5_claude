#pragma once

// ── Debug Ausgabe ──────────────────────────────────────────
#define _SERIAL_LOG
//#define _BT_LOG

// ── Kommunikationskanal (genau einen aktivieren) ────────────
// Aktiv: comm nutzt Serial1 (BT-UART). Auskommentiert: comm nutzt Serial (USB).
//#define COMM_USE_BLUETOOTH

// ── Test-Modi (auskommentieren = deaktiviert) ──────────────
// #define TEST_MOTORS
// #define TEST_MOTORS_SINGLE  // ← einzelne Motoren testen
// #define TEST_BAROMETER
// #define TEST_KEYBOARD
// #define TEST_I2C_SCAN
// #define TEST_IMU
// #define TEST_ULTRASONIC

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
#define EEPROM_ADDR_KP 0
#define EEPROM_ADDR_KI 4
#define EEPROM_ADDR_KD 8
#define EEPROM_SIZE 16

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
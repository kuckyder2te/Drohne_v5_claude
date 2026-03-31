#pragma once

// ── Test-Modi (auskommentieren = deaktiviert) ──────────────
// #define TEST_MOTORS
// #define TEST_BAROMETER
// #define TEST_I2C_SCAN
// #define TEST_KEYBOARD

// ESC PWM-Parameter (Standard: 1000–2000 µs)
#define ESC_MIN_US    1000
#define ESC_MAX_US    2000
#define ESC_FREQ_HZ   50

// ── Bluetooth HC-06 (UART0) ────────────────────────────────
#define BT_UART       Serial1
#define BT_BAUD       9600

// ── Flugparameter ──────────────────────────────────────────
#define TARGET_HEIGHT_CM   50
#define MAX_HEIGHT_CM      100
#define THROTTLE_STEP      50

// ── EEPROM-Adressen ────────────────────────────────────────
#define EEPROM_ADDR_KP   0
#define EEPROM_ADDR_KI   4
#define EEPROM_ADDR_KD   8
#define EEPROM_SIZE      16

// ── PID Startwerte ─────────────────────────────────────────
#define PID_KP_DEFAULT   1.0f
#define PID_KI_DEFAULT   0.05f
#define PID_KD_DEFAULT   0.1f

// ── Regelkreis ─────────────────────────────────────────────
#define PID_INTERVAL_MS  50     // 20 Hz Regelfrequenz
#define THROTTLE_MIN_CM  5      // Untergrenze Zielhöhe
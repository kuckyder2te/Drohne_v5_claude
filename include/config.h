#pragma once

// ── Test-Modi (auskommentieren = deaktiviert) ──────────────
// #define TEST_MOTORS
#define TEST_BAROMETER

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
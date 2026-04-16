#pragma once

// ── Motoren (PWM) ──────────────────────────────────────────
#define PIN_MOTOR_FL 11 // Front Left
#define PIN_MOTOR_FR 12 // Front Right
#define PIN_MOTOR_BL 14 // Back Left
#define PIN_MOTOR_BR 13 // Back Right

// I²C Pins
#define PIN_SDA 4
#define PIN_SCL 5 
#define PIN_IMU_INT 21       

// Bluetooth HC-06 (UART0)
#define PIN_BT_TX 0
#define PIN_BT_RX 1

// ── SPI (NRF24) ─────────────────────────────────
#define PIN_NRF_MOSI 19 
#define PIN_NRF_MISO 16
#define PIN_NRF_SCK 18
#define PIN_NRF_CSN 17
#define PIN_NRF_CE 20

// ── Sonstige Pins ────────────────────────────────

#define BUZZER 10
#define BATTERY 26

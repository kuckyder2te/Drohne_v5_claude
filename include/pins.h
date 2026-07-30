#pragma once

// ── Motoren (PWM) ──────────────────────────────────────────
#define PIN_MOTOR_FL 12 // Front Left  yellow
#define PIN_MOTOR_FR 13 // Front Right  green
#define PIN_MOTOR_BR 14 // Back Right blue
#define PIN_MOTOR_BL 15 // Back Left white

// I²C Pins
#define PIN_SDA 4
#define PIN_SCL 5 
#define PIN_NRF_INT 21     
#define PIN_IMU_INT 3   // ← ICM-20948 INT Pin

// Bluetooth HC-06 (UART0)
#define PIN_BT_TX 0
#define PIN_BT_RX 1

// ── SPI (NRF24) ─────────────────────────────────
#define PIN_NRF_MOSI 19 
#define PIN_NRF_MISO 16
#define PIN_NRF_SCK 18
#define PIN_NRF_CSN 17
#define PIN_NRF_CE 20
#define PIN_NRF_INT 21   

// ── Ultrasonic  HC-SR04──────────────────────────
#define PIN_ULTRASONIC_TRIG1 8
#define PIN_ULTRASONIC_ECHO1 6
// #define PIN_ULTRASONIC_TRIG1 9  // für späteren Einsatz eines zweiten Sensors reserviert
// #define PIN_ULTRASONIC_ECHO2 7  

// ── Sonstige Pins ────────────────────────────────
#define BUZZER 10
#define BATTERY 26  // ADC0




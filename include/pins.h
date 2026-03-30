  #pragma once

// ── Motoren (PWM) ──────────────────────────────────────────
#define PIN_MOTOR_FL  11   // Front Left
#define PIN_MOTOR_FR  12   // Front Right
#define PIN_MOTOR_BL  14   // Back Left
#define PIN_MOTOR_BR  13   // Back Right

// I²C Pins
#define PIN_SDA 4
#define PIN_SCL 5

// Radio
#define PIN_RADIO_CE 20
#define PIN_RADIO_CSN 17

// UART
#define PIN_BT_TX     0    // ⚠️ noch nicht in pins.h — bitte prüfen!
#define PIN_BT_RX     1    // ⚠️ noch nicht in pins.h — bitte prüfen!

// ── SPI (MPU9250 + NRF24) ─────────────────────────────────
#define PIN_SPI_MOSI  19   // ⚠️ noch nicht in pins.h — bitte prüfen!
#define PIN_SPI_MISO  16   // ⚠️ noch nicht in pins.h — bitte prüfen!
#define PIN_SPI_SCK   18   // ⚠️ noch nicht in pins.h — bitte prüfen!
#define PIN_IMU_CS    9    // ⚠️ noch nicht in pins.h — bitte prüfen!
#define PIN_NRF_CE    20
#define PIN_NRF_CSN   17

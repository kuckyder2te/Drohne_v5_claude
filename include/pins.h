#pragma once

// ── Motoren (PWM) ──────────────────────────────────────────
#define PIN_MOTOR_FL 12 // Front Left  yellow
#define PIN_MOTOR_FR 13 // Front Right  green
#define PIN_MOTOR_BR 14 // Back Right blue
#define PIN_MOTOR_BL 15 // Back Left white
#define PIN_ESC_POWER 28

// Farbmarkierung am ESC
// 1 (A) (weisser Punkt) black
// 2 (B)                green
// 3 (C)                yellow

// ── ESC-Stromversorgung ──────────────────────────
// GP28 (Board-Pin 34) schaltet ueber einen Logic-Level-N-FET (IRLZ44N,
// Low-Side) die LiPo-Masse der vier ESCs. HIGH = ESCs am Strom.
//
// WICHTIG (Hardware): Zwischen Gate und Masse gehoert ein Pulldown (~10 k).
// Vom Reset bis zum ersten pinMode() ist der GPIO hochohmig; ohne Pulldown
// haengt das Gate in der Luft und die ESCs koennen waehrend Reset, BOOTSEL
// oder Flashen unkontrolliert Strom bekommen.
// WICHTIG Die Masse-Kabel (braun) der ESCs dürfen nicht mit der Masse des Mainboards verbunden werden, 
// da sonst der die ESCs sofort Strom bekommen, sobald die LiPo-Spannung anliegt. Die ESCs müssen über den PIN_ESC_POWER geschaltet werden.
// Die 5V (rot) werden nicht benötigt


#define PIN_SDA 4
#define PIN_SCL 5     
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
// Zwei Sensoren, beide nach unten gerichtet. Gemessen wird abwechselnd je ein
// Sensor pro Ultrasonic::update(), fusioniert wird das Minimum - siehe
// lib/Ultrasonic/Ultrasonic.h.
#define PIN_ULTRASONIC_TRIG1 7 // yellow
#define PIN_ULTRASONIC_ECHO1 6 // blue
#define PIN_ULTRASONIC_TRIG2 9 // green
#define PIN_ULTRASONIC_ECHO2 8 // purple

// ── Sonstige Pins ────────────────────────────────
#define BUZZER 10
#define BATTERY 26  // ADC0






#include <Arduino.h>
#include "config.h"
#include "control/MotorMixer.h"
#include "control/PIDController.h"
#include "sensor/Barometer.h"
#include "comm/KeyboardInput.h"

MotorMixer    motors;
Barometer     baro;
KeyboardInput keyboard;
PIDController pid(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);

// ── Zustandsvariablen ──────────────────────────────────────
float    targetHeightCm = 0.0f;   // Zielhöhe
bool     armed          = false;  // Motoren aktiv?
uint32_t lastPidMs      = 0;
uint32_t lastPrintMs    = 0;

// ── Test-Modi ──────────────────────────────────────────────
#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;
void printMotorHelp() {
    Serial.println("─────────────────────────────");
    Serial.println(" MOTORTEST: + - s h");
    Serial.println("─────────────────────────────");
}
#endif

// ── Hilfsfunktionen ────────────────────────────────────────
void i2cScan() {
    Serial.println("[I2C] Scanne Bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Gefunden: 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println("[I2C] Kein Gerät!");
}

void printHelp() {
    Serial.println("────────────────────────────────────");
    Serial.println(" Pfeil hoch  = Zielhöhe +10 cm");
    Serial.println(" Pfeil runter = Zielhöhe -10 cm");
    Serial.println(" a = ARM (Motoren ein)");
    Serial.println(" s = DISARM (Motoren aus)");
    Serial.println(" r = Barometer rekalibrieren");
    Serial.println(" h = Hilfe");
    Serial.println("────────────────────────────────────");
}

void disarm() {
    armed = false;
    targetHeightCm = 0.0f;
    motors.stop();
    pid.reset();
    Serial.println("[CTRL] DISARM — Motoren gestoppt");
}

// ── Setup ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DROHNE PICO BOOT ===");

#ifdef TEST_I2C_SCAN
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    i2cScan();
#endif

#ifdef TEST_MOTORS
    Serial.println(">> Modus: MOTORTEST");
    printMotorHelp();
    motors.begin();
#endif

#ifdef TEST_BAROMETER
    Serial.println(">> Modus: BAROMETER TEST");
    if (!baro.begin()) {
        Serial.println("FEHLER: Programm gestoppt.");
        while (true) delay(1000);
    }
#endif

#ifdef TEST_KEYBOARD
    Serial.println(">> Modus: KEYBOARD TEST");
    if (!baro.begin()) {
        Serial.println("FEHLER: Barometer nicht gefunden.");
        while (true) delay(1000);
    }
    keyboard.begin();
    printHelp();
#endif

#ifndef TEST_MOTORS
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD
    // ── Normalbetrieb ──────────────────────────────────────
    Serial.println(">> Modus: NORMALBETRIEB");

    if (!baro.begin()) {
        Serial.println("FEHLER: Barometer! Programm gestoppt.");
        while (true) delay(1000);
    }

    motors.begin();
    pid.begin();
    keyboard.begin();
    printHelp();

    Serial.println("[CTRL] Bereit — 'a' zum Armen");
#endif
#endif
#endif
}

// ── Loop ───────────────────────────────────────────────────
void loop() {

    // ── TEST_MOTORS ────────────────────────────────────────
#ifdef TEST_MOTORS
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case '+': currentThrottle += THROTTLE_STEP; motors.setThrottle(currentThrottle); break;
            case '-': currentThrottle -= THROTTLE_STEP; motors.setThrottle(currentThrottle); break;
            case 's': case 'S': currentThrottle = ESC_MIN_US; motors.stop(); break;
            case 'h': case 'H': printMotorHelp(); break;
        }
    }
#endif

    // ── TEST_BAROMETER ─────────────────────────────────────
#ifdef TEST_BAROMETER
    baro.update();
    Serial.print("[BARO] Höhe: ");
    Serial.print(baro.getAltitudeCm(), 1);
    Serial.print(" cm  |  Druck: ");
    Serial.print(baro.getPressure(), 2);
    Serial.print(" Pa  |  Temp: ");
    Serial.print(baro.getTemperature(), 1);
    Serial.println(" °C");
    delay(500);
#endif

    // ── TEST_KEYBOARD ──────────────────────────────────────
#ifdef TEST_KEYBOARD
    baro.update();
    KeyEvent key = keyboard.getKey();
    switch (key) {
        case KeyEvent::ARROW_UP:   Serial.println("[KEY] Pfeil HOCH → Throttle +");   break;
        case KeyEvent::ARROW_DOWN: Serial.println("[KEY] Pfeil RUNTER → Throttle -"); break;
        case KeyEvent::KEY_S:      Serial.println("[KEY] STOP");                       break;
        case KeyEvent::KEY_R:      baro.calibrate();                                   break;
        case KeyEvent::KEY_H:      printHelp();                                        break;
        case KeyEvent::NONE:       break;
    }
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.print("[BARO] Höhe: ");
        Serial.print(baro.getAltitudeCm(), 1);
        Serial.println(" cm");
    }
#endif

    // ── NORMALBETRIEB ──────────────────────────────────────
#ifndef TEST_MOTORS
#ifndef TEST_BAROMETER
#ifndef TEST_KEYBOARD

    // Barometer immer aktualisieren
    baro.update();

    // Tastatureingabe
    KeyEvent key = keyboard.getKey();
    switch (key) {
        case KeyEvent::ARROW_UP:
            targetHeightCm = constrain(targetHeightCm + 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
            Serial.print("[CTRL] Zielhöhe: ");
            Serial.print(targetHeightCm);
            Serial.println(" cm");
            break;

        case KeyEvent::ARROW_DOWN:
            targetHeightCm = constrain(targetHeightCm - 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
            Serial.print("[CTRL] Zielhöhe: ");
            Serial.print(targetHeightCm);
            Serial.println(" cm");
            break;

        case KeyEvent::KEY_S:
            disarm();
            break;

        case KeyEvent::KEY_R:
            baro.calibrate();
            pid.reset();
            break;

        case KeyEvent::KEY_H:
            printHelp();
            break;

        case KeyEvent::NONE:
            break;
    }

    // ARM per 'a' Taste
    if (Serial.available()) {
        char c = Serial.read();
        if ((c == 'a' || c == 'A') && !armed) {
            armed = true;
            targetHeightCm = 20.0f;  // Startziel: 20 cm
            pid.reset();
            Serial.println("[CTRL] ARM — Ziel: 20 cm");
        }
    }

    // PID-Regelkreis — läuft mit PID_INTERVAL_MS
    if (armed && (millis() - lastPidMs >= PID_INTERVAL_MS)) {
        lastPidMs = millis();

        float currentHeight = baro.getAltitudeCm();
        float throttle      = pid.compute(targetHeightCm, currentHeight);
        motors.setThrottle((uint16_t)throttle);
    }

    // Statusausgabe alle 500ms
    if (millis() - lastPrintMs >= 500) {
        lastPrintMs = millis();
        Serial.print("[CTRL] Ziel: ");
        Serial.print(targetHeightCm, 1);
        Serial.print(" cm | Ist: ");
        Serial.print(baro.getAltitudeCm(), 1);
        Serial.print(" cm | Armed: ");
        Serial.println(armed ? "JA" : "NEIN");
    }

#endif
#endif
#endif
}
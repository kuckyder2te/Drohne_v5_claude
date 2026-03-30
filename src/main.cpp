/*
 * Hauptprogramm für Drohne Pico
 * Author: Claude & Kucky
 * Date: 28.03.2026
 * Description: Hauptprogramm für Drohne Pico.
 * 
 * Je nach Kompilier-Flag können verschiedene Testmodi aktiviert werden:
 * - TEST_MOTORS: Testet die Motoransteuerung über die serielle Schnittstelle.
 * - TEST_BAROMETER: Testet die Ausgabe von Barometerdaten.
 * - TEST_KEYBOARD: Testet die Erkennung von Tastatureingaben.
 * 
 * Nur ein Modus sollte gleichzeitig aktiviert sein, um Konflikte zu vermeiden.
 */

#include <Arduino.h>
#include "config.h"
#include "control/MotorMixer.h"
#include "sensor/Barometer.h"
#include "comm/KeyboardInput.h"

MotorMixer    motors;
Barometer     baro;
KeyboardInput keyboard;

#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;

void printHelp() {
    Serial.println("─────────────────────────────");
    Serial.println(" MOTORTEST Befehle:");
    Serial.println("  + = Throttle +50 µs");
    Serial.println("  - = Throttle -50 µs");
    Serial.println("  s = STOP");
    Serial.println("─────────────────────────────");
}
#endif

#ifdef TEST_KEYBOARD
void printKeyHelp() {
    Serial.println("─────────────────────────────────");
    Serial.println(" TASTATURTEST:");
    Serial.println("  Pfeil hoch  = Throttle +");
    Serial.println("  Pfeil runter = Throttle -");
    Serial.println("  s = Stop");
    Serial.println("  r = Barometer rekalibrieren");
    Serial.println("  h = Diese Hilfe");
    Serial.println("─────────────────────────────────");
}
#endif

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DROHNE PICO BOOT ===");

#ifdef TEST_MOTORS
    Serial.println(">> Modus: MOTORTEST");
    printHelp();
    motors.begin();
#endif

#ifdef TEST_BAROMETER
    Serial.println(">> Modus: BAROMETER TEST");
    if (!baro.begin()) {
        Serial.println("FEHLER: Programm gestoppt.");
        while (true) delay(1000);
    }
    Serial.println("Bereit. Ausgabe alle 500ms:");
#endif

#ifdef TEST_KEYBOARD
    Serial.println(">> Modus: KEYBOARD TEST");
    if (!baro.begin()) {
        Serial.println("FEHLER: Barometer nicht gefunden.");
        while (true) delay(1000);
    }
    keyboard.begin();
    printKeyHelp();
#endif
}

void i2cScan() {
    Serial.println("[I2C] Scanne Bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Gerät gefunden: 0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println("[I2C] Kein Gerät gefunden!");
}

void loop() {
#ifdef TEST_MOTORS
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case '+': currentThrottle += THROTTLE_STEP; motors.setThrottle(currentThrottle); break;
            case '-': currentThrottle -= THROTTLE_STEP; motors.setThrottle(currentThrottle); break;
            case 's': case 'S': currentThrottle = ESC_MIN_US; motors.stop(); break;
            case 'h': case 'H': printHelp(); break;
        }
    }
#endif

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

#ifdef TEST_KEYBOARD
    // Barometer laufend aktualisieren
    baro.update();

    // Tastatureingabe prüfen
    KeyEvent key = keyboard.getKey();
    switch (key) {
        case KeyEvent::ARROW_UP:
            Serial.println("[KEY] Pfeil HOCH → Throttle +");
            break;
        case KeyEvent::ARROW_DOWN:
            Serial.println("[KEY] Pfeil RUNTER → Throttle -");
            break;
        case KeyEvent::KEY_S:
            Serial.println("[KEY] STOP");
            break;
        case KeyEvent::KEY_R:
            Serial.println("[KEY] Rekalibrierung...");
            baro.calibrate();
            break;
        case KeyEvent::KEY_H:
            printKeyHelp();
            break;
        case KeyEvent::NONE:
            break;
    }

    // Höhe alle 500ms ausgeben
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.print("[BARO] Höhe: ");
        Serial.print(baro.getAltitudeCm(), 1);
        Serial.println(" cm");
    }
#endif
}
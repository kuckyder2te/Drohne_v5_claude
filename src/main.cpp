#include <Arduino.h>
#include "config.h"
#include "control/MotorMixer.h"
#include "sensor/Barometer.h"

MotorMixer motors;
Barometer  baro;

#ifdef TEST_MOTORS
uint16_t currentThrottle = ESC_MIN_US;

void printHelp() {
    Serial.println("─────────────────────────────");
    Serial.println(" MOTORTEST Befehle:");
    Serial.println("  + = Throttle +50 µs");
    Serial.println("  - = Throttle -50 µs");
    Serial.println("  s = STOP (1000 µs)");
    Serial.println("  h = Diese Hilfe");
    Serial.println("─────────────────────────────");
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
}
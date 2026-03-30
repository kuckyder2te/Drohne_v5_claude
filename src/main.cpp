#include <Arduino.h>
#include "config.h"
#include "control/MotorMixer.h"

MotorMixer motors;
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

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DROHNE PICO — Schritt 2: Motortest ===");
    printHelp();
    motors.begin();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case '+':
                currentThrottle += THROTTLE_STEP;
                motors.setThrottle(currentThrottle);
                break;
            case '-':
                currentThrottle -= THROTTLE_STEP;
                motors.setThrottle(currentThrottle);
                break;
            case 's':
            case 'S':
                currentThrottle = ESC_MIN_US;
                motors.stop();
                break;
            case 'h':
            case 'H':
                printHelp();
                break;
            default:
                break;
        }
    }
}
#include "comm/KeyboardInput.h"

void KeyboardInput::begin() {
    // Serial bereits in main.cpp gestartet
    Serial.println("[KEY] Tastatur bereit");
    Serial.println("[KEY] Pfeiltasten: hoch/runter | s=Stop | r=Rekalibrierung | h=Hilfe");
}

KeyEvent KeyboardInput::getKey() {
    while (Serial.available()) {
        uint8_t c = Serial.read();

        switch (_escState) {
            case 0:  // Normaler Zustand
                if (c == 0x1B) {
                    _escState = 1;  // ESC empfangen
                } else {
                    // Normale Tasten
                    switch (c) {
                        case 's': case 'S': return KeyEvent::KEY_S;
                        case 'h': case 'H': return KeyEvent::KEY_H;
                        case 'r': case 'R': return KeyEvent::KEY_R;
                    }
                }
                break;

            case 1:  // ESC empfangen, warte auf [
                if (c == 0x5B) {
                    _escState = 2;
                } else {
                    _escState = 0;  // Kein gültiges Escape — reset
                }
                break;

            case 2:  // ESC [ empfangen, warte auf Pfeil
                _escState = 0;  // Reset für nächste Sequenz
                switch (c) {
                    case 0x41: return KeyEvent::ARROW_UP;
                    case 0x42: return KeyEvent::ARROW_DOWN;
                }
                break;
        }
    }
    return KeyEvent::NONE;
}
#include "comm/SerialInput.h"

void SerialInput::begin() {
    _escState = 0;
}

KeyEvent SerialInput::getKey() {
    while (Serial.available()) {
        uint8_t c = Serial.read();
        KeyEvent ev = _parseChar(c);
        if (ev != KeyEvent::NONE) return ev;
    }
    return KeyEvent::NONE;
}

KeyEvent SerialInput::_parseChar(uint8_t c) {
    switch (_escState) {
    case 0:
        if (c == 0x1B) {
            _escState = 1;
        } else {
            switch (toupper(c)) {
                case 'A': return KeyEvent::KEY_A;
                case 'S': return KeyEvent::KEY_S;
                case 'H': return KeyEvent::KEY_H;
                case 'R': return KeyEvent::KEY_R;
                case 'L': return KeyEvent::KEY_L;
                case '+': return KeyEvent::ARROW_UP;
                case '-': return KeyEvent::ARROW_DOWN;
            }
        }
        break;
    case 1:
        _escState = (c == 0x5B) ? 2 : 0;
        break;
    case 2:
        _escState = 0;
        switch (c) {
            case 0x41: return KeyEvent::ARROW_UP;
            case 0x42: return KeyEvent::ARROW_DOWN;
        }
        break;
    }
    return KeyEvent::NONE;
}

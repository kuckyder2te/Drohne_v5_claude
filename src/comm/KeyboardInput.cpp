#include "comm/KeyboardInput.h"

void KeyboardInput::begin() {
    _escState  = 0;
    _btBuffer  = "";
    _btCommand = "";
}

KeyEvent KeyboardInput::getKey() {
    _btCommand = "";  // Reset letzten BT Befehl

    // ── USB Serial lesen ──────────────────────────────
    while (Serial.available()) {
        uint8_t c = Serial.read();
        KeyEvent ev = _parseChar(c, false);
        if (ev != KeyEvent::NONE) return ev;
    }

    // ── BT Serial lesen ───────────────────────────────
    while (BT_UART.available()) {
        uint8_t c = BT_UART.read();

        // Zeilenende → Befehl fertig
        if (c == '\n' || c == '\r') {
            if (_btBuffer.length() > 0) {
                // Einzelne Zeichen als KeyEvent
                if (_btBuffer.length() == 1) {
                    char ch = toupper(_btBuffer[0]);
                    _btBuffer = "";
                    switch (ch) {
                        case 'A': return KeyEvent::KEY_A;
                        case 'S': return KeyEvent::KEY_S;
                        case 'H': return KeyEvent::KEY_H;
                        case 'R': return KeyEvent::KEY_R;
                        case '+': return KeyEvent::ARROW_UP;
                        case '-': return KeyEvent::ARROW_DOWN;
                        default:  _btCommand = String(ch); return KeyEvent::NONE;
                    }
                }
                // Mehrzeichen → BT Befehl (P=x, I=x etc.)
                _btCommand = _btBuffer;
                _btBuffer  = "";
            }
            return KeyEvent::NONE;
        }

        // Puffer füllen
        _btBuffer += (char)c;
        if (_btBuffer.length() > 20) _btBuffer = "";
    }

    return KeyEvent::NONE;
}

String KeyboardInput::getBTCommand() {
    return _btCommand;
}

KeyEvent KeyboardInput::_parseChar(uint8_t c, bool fromBT) {
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

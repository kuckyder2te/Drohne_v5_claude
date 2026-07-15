#include "comm/SerialInput.h"

KeyEvent SerialInput::getKey() {
    _cmd = "";
    while (Serial.available()) {
        uint8_t c = Serial.read();

        // ANSI-Escape-Sequenzen verwerfen (funktionieren in pio monitor nicht zuverlaessig)
        if (_escSt == 0 && c == 0x1B) { _escSt = 1; continue; }
        if (_escSt == 1) { _escSt = (c == '[') ? 2 : 0; continue; }
        if (_escSt == 2) { _escSt = 0; continue; }

        // Sofortbefehle (kein Enter noetig)
        if (c == '+') { _buf = ""; return KeyEvent::ARROW_UP; }
        if (c == '-') { _buf = ""; return KeyEvent::ARROW_DOWN; }
        if (c == 'd' || c == 'D') { _buf = ""; return KeyEvent::KEY_D; }


        if (c == '\r' || c == '\n') {
            if (_buf.length() == 0) continue;

            if (_buf.length() == 1) {
                char ch = toupper(_buf[0]);
                _buf = "";
                switch (ch) {
                    case 'A': return KeyEvent::KEY_A;
                    case 'D': return KeyEvent::KEY_D;
                    case 'H': return KeyEvent::KEY_H;
                    case 'R': return KeyEvent::KEY_R;
                    case 'L': return KeyEvent::KEY_L;
                    default:  _cmd = String(ch); return KeyEvent::NONE;
                }
            } else {
                _cmd = _buf;
                _buf = "";
                return KeyEvent::NONE;
            }
        } else if (c >= 0x20) {
            _buf += (char)c;
            if (_buf.length() > 50) _buf = "";
        }
    }
    return KeyEvent::NONE;
}

#include "comm/CommChannel.h"
#include "config.h"

void CommChannel::send(const char* msg) {
    _stream.print(msg);
}

void CommChannel::sendLine(const char* msg) {
    _stream.println(msg);
}

void CommChannel::printHelp() {
    sendLine("------------------------------------");
    sendLine(" a   = ARM (2x bestaetigen)");
    sendLine(" d   = DISARM (sofort)");
    sendLine(" +/- = Hoehe +/-10 cm (sofort)");
    sendLine(" r   = Baro rekalibrieren");
    sendLine(" l   = Statuslog ein/aus");
    sendLine(" h   = Hilfe");
    sendLine(" ?   = PID-Werte anzeigen");
    sendLine(" PID: P=x.x  I=x.x  D=x.x");
    sendLine(" SAVE / RESET");
    sendLine("------------------------------------");
}

KeyEvent CommChannel::resolveSingleChar(char ch) {
    switch (toupper(ch)) {
        case 'A': return KeyEvent::KEY_A;
        case 'D': return KeyEvent::KEY_D;
        case 'H': return KeyEvent::KEY_H;
        case 'R': return KeyEvent::KEY_R;
        case 'L': return KeyEvent::KEY_L;
        default:  _command = String((char)toupper(ch)); return KeyEvent::NONE;
    }
}

KeyEvent CommChannel::getKey() {
    _command = "";

    // Einzelzeichen ohne Newline: nach 200 ms als Befehl werten (deckt Quellen ohne
    // Enter ab, z.B. BT-App-Buttons). Mehrzeichenbefehle (PID-Werte, RESET) werden
    // ausschliesslich per Newline abgeschlossen.
    if (_buffer.length() == 1 && (millis() - _lastCharMs) > 200) {
        KeyEvent ev = resolveSingleChar(_buffer[0]);
        _buffer = "";
        return ev;
    }

    while (_stream.available()) {
        uint8_t c = _stream.read();

        // ANSI-Escape-Sequenzen verwerfen (z.B. Pfeiltasten ueber Terminal, hier nicht unterstuetzt)
        if (_escSt == 0 && c == 0x1B) { _escSt = 1; continue; }
        if (_escSt == 1) { _escSt = (c == '[') ? 2 : 0; continue; }
        if (_escSt == 2) { _escSt = 0; continue; }

        if (c == '+') { _buffer = ""; return KeyEvent::ARROW_UP; }
        if (c == '-') { _buffer = ""; return KeyEvent::ARROW_DOWN; }

        if (c == '\r' || c == '\n') {
            if (_buffer.length() == 0) continue;

            if (_buffer.length() == 1) {
                KeyEvent ev = resolveSingleChar(_buffer[0]);
                _buffer = "";
                return ev;
            } else {
                _command = _buffer;
                _buffer = "";
                return KeyEvent::NONE;
            }
        } else if (c >= 0x20) {
            _lastCharMs = millis();
            _buffer += (char)c;
            if (_buffer.length() > 100) _buffer = "";
        }
    }

    return KeyEvent::NONE;
}

void CommChannel::processCommand(const String &cmd,
                                 PIDController &pidHeight,
                                 PIDController &pidRoll,
                                 PIDController &pidPitch,
                                 Settings &settings) {
    if (cmd.length() == 0) return;

    if (cmd == "?") {
        sendLine("[PID] === Hoehe ===");
        char buf[40];
        snprintf(buf, sizeof(buf), "[PID] Kp=%.4f", pidHeight.getKp()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Ki=%.4f", pidHeight.getKi()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Kd=%.4f", pidHeight.getKd()); sendLine(buf);
        sendLine("[PID] === Roll ===");
        snprintf(buf, sizeof(buf), "[PID] Kp=%.4f", pidRoll.getKp()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Ki=%.4f", pidRoll.getKi()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Kd=%.4f", pidRoll.getKd()); sendLine(buf);
        sendLine("[PID] === Pitch ===");
        snprintf(buf, sizeof(buf), "[PID] Kp=%.4f", pidPitch.getKp()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Ki=%.4f", pidPitch.getKi()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[PID] Kd=%.4f", pidPitch.getKd()); sendLine(buf);
        return;
    }

    String ucmd = cmd;
    ucmd.toUpperCase();

    if (ucmd == "SAVE" || ucmd == "S") {
        settings.save(pidHeight.getKp(), pidHeight.getKi(), pidHeight.getKd());
        sendLine("[BT] PID Hoehe gespeichert");
        return;
    }
    if (ucmd == "RESET") {
        settings.reset();
        pidHeight.setKp(PID_KP_HEIGHT);
        pidHeight.setKi(PID_KI_HEIGHT);
        pidHeight.setKd(PID_KD_HEIGHT);
        sendLine("[BT] PID auf Standard zurueckgesetzt");
        return;
    }

    char p0 = ucmd[0];
    char p1 = ucmd.length() > 1 ? ucmd[1] : 0;
    char buf[50];

    // Komma als Dezimaltrennzeichen akzeptieren (deutsche Tastatur)
    ucmd.replace(',', '.');

    if (p0 == 'R' && p1 == 'P' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidRoll.setKp(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Kp=%.3f", v); sendLine(buf);
    } else if (p0 == 'R' && p1 == 'I' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidRoll.setKi(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Ki=%.3f", v); sendLine(buf);
    } else if (p0 == 'R' && p1 == 'D' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidRoll.setKd(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Kd=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'P' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidPitch.setKp(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Kp=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'I' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidPitch.setKi(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Ki=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'D' && ucmd.length() > 2 && ucmd[2] == '=') {
        float v = ucmd.substring(3).toFloat();
        pidPitch.setKd(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Kd=%.3f", v); sendLine(buf);
    } else if (ucmd.length() >= 3 && p1 == '=') {
        float value = ucmd.substring(2).toFloat();
        switch (p0) {
            case 'P':
                pidHeight.setKp(value);
                snprintf(buf, sizeof(buf), "[BT] Hoehe Kp=%.3f", value); sendLine(buf);
                break;
            case 'I':
                pidHeight.setKi(value);
                snprintf(buf, sizeof(buf), "[BT] Hoehe Ki=%.3f", value); sendLine(buf);
                break;
            case 'D':
                pidHeight.setKd(value);
                snprintf(buf, sizeof(buf), "[BT] Hoehe Kd=%.3f", value); sendLine(buf);
                break;
            default:
                snprintf(buf, sizeof(buf), "[BT] Unbekannt: %s", cmd.c_str());
                sendLine(buf);
        }
    } else {
        snprintf(buf, sizeof(buf), "[BT] Unbekannt: %s", cmd.c_str());
        sendLine(buf);
    }
}

#include "comm/BluetoothComm.h"
#include "myLogger.h"
#include "config.h"
#include "pins.h"

void BluetoothComm::send(const char* msg) {
    BT_UART.print(msg);
}

void BluetoothComm::sendLine(const char* msg) {
    BT_UART.println(msg);
}

void BluetoothComm::begin() {
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    BT_UART.begin(BT_BAUD);
    sendLine("[BT] Drohne bereit");
    sendLine("[BT] Befehle: A D R L H  +/-");
    sendLine("[BT] PID: P=x I=x D=x  RP= RI= RD=  PP= PI= PD=");
    sendLine("[BT] S=Speichern  RESET  ?=Abfrage");
    LOG("[BT] Bluetooth bereit");
}

KeyEvent BluetoothComm::getKey() {
    _command = "";

    // Timeout: nach 200 ms ohne weiteres Zeichen Befehl ausfuehren (kein Newline noetig).
    // Nur bekannte Steuerzeichen und Einzelbefehle (?, S) werden per Timeout dispatcht.
    // Unbekannte Zeichen (PID-Prefixe wie P, I, D) bleiben im Buffer bis Newline.
    if (_buffer.length() > 0 && (millis() - _lastCharMs) > 200) {
        if (_buffer.length() == 1) {
            char ch = toupper(_buffer[0]);
            switch (ch) {
                case 'A': _buffer = ""; return KeyEvent::KEY_A;
                case 'D': _buffer = ""; return KeyEvent::KEY_D;
                case 'H': _buffer = ""; return KeyEvent::KEY_H;
                case 'R': _buffer = ""; return KeyEvent::KEY_R;
                case 'L': _buffer = ""; return KeyEvent::KEY_L;
                case '+': _buffer = ""; return KeyEvent::ARROW_UP;
                case '-': _buffer = ""; return KeyEvent::ARROW_DOWN;
                case '?': _buffer = ""; _command = "?"; return KeyEvent::NONE;
                case 'S': _buffer = ""; _command = "S"; return KeyEvent::NONE;
                default:  return KeyEvent::NONE; // im Buffer lassen, auf Newline warten
            }
        } else {
            _command = _buffer;
            _buffer = "";
            return KeyEvent::NONE;
        }
    }

    while (BT_UART.available()) {
        uint8_t c = BT_UART.read();

        if (c == '\r' || c == '\n') {
            if (_buffer.length() == 0) continue;

            if (_buffer.length() == 1) {
                char ch = toupper(_buffer[0]);
                _buffer = "";
                switch (ch) {
                    case 'A': return KeyEvent::KEY_A;
                    case 'D': return KeyEvent::KEY_D;
                    case 'H': return KeyEvent::KEY_H;
                    case 'R': return KeyEvent::KEY_R;
                    case 'L': return KeyEvent::KEY_L;
                    case '+': return KeyEvent::ARROW_UP;
                    case '-': return KeyEvent::ARROW_DOWN;
                    default:  _command = String(ch); return KeyEvent::NONE;
                }
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

String BluetoothComm::getCommand() {
    return _command;
}

void BluetoothComm::processCommand(const String &cmd,
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

    if (p0 == 'R' && p1 == 'P' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidRoll.setKp(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Kp=%.3f", v); sendLine(buf);
    } else if (p0 == 'R' && p1 == 'I' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidRoll.setKi(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Ki=%.3f", v); sendLine(buf);
    } else if (p0 == 'R' && p1 == 'D' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidRoll.setKd(v);
        snprintf(buf, sizeof(buf), "[BT] Roll Kd=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'P' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidPitch.setKp(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Kp=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'I' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidPitch.setKi(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Ki=%.3f", v); sendLine(buf);
    } else if (p0 == 'P' && p1 == 'D' && cmd.length() > 2 && cmd[2] == '=') {
        float v = cmd.substring(3).toFloat();
        pidPitch.setKd(v);
        snprintf(buf, sizeof(buf), "[BT] Pitch Kd=%.3f", v); sendLine(buf);
    } else if (cmd.length() >= 3 && p1 == '=') {
        float value = cmd.substring(2).toFloat();
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

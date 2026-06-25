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
    sendLine("[BT] SAVE RESET  ?");
    LOG("[BT] Bluetooth bereit");
}

KeyEvent BluetoothComm::getKey() {
    _command = "";

    // Timeout: nach 200 ms ohne weiteres Zeichen Befehl ausfuehren (kein Newline noetig)
    // Unbekannte Einzelzeichen (z.B. 'P' als Anfang von "P=5.0") bleiben im Buffer
    // und werden erst bei Newline dispatched.
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
        LOG("[PID] === Hoehe ===");
        LOG_FMT("[PID] Kp=%.4f", pidHeight.getKp());
        LOG_FMT("[PID] Ki=%.4f", pidHeight.getKi());
        LOG_FMT("[PID] Kd=%.4f", pidHeight.getKd());
        LOG("[PID] === Roll ===");
        LOG_FMT("[PID] Kp=%.4f", pidRoll.getKp());
        LOG_FMT("[PID] Ki=%.4f", pidRoll.getKi());
        LOG_FMT("[PID] Kd=%.4f", pidRoll.getKd());
        LOG("[PID] === Pitch ===");
        LOG_FMT("[PID] Kp=%.4f", pidPitch.getKp());
        LOG_FMT("[PID] Ki=%.4f", pidPitch.getKi());
        LOG_FMT("[PID] Kd=%.4f", pidPitch.getKd());
        return;
    }

    String ucmd = cmd;
    ucmd.toUpperCase();

    if (ucmd == "SAVE") {
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

    if (p0 == 'R' && p1 == 'P' && cmd.length() > 2 && cmd[2] == '=') {
        pidRoll.setKp(cmd.substring(3).toFloat());
    } else if (p0 == 'R' && p1 == 'I' && cmd.length() > 2 && cmd[2] == '=') {
        pidRoll.setKi(cmd.substring(3).toFloat());
    } else if (p0 == 'R' && p1 == 'D' && cmd.length() > 2 && cmd[2] == '=') {
        pidRoll.setKd(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'P' && cmd.length() > 2 && cmd[2] == '=') {
        pidPitch.setKp(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'I' && cmd.length() > 2 && cmd[2] == '=') {
        pidPitch.setKi(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'D' && cmd.length() > 2 && cmd[2] == '=') {
        pidPitch.setKd(cmd.substring(3).toFloat());
    } else if (cmd.length() >= 3 && p1 == '=') {
        float value = cmd.substring(2).toFloat();
        switch (p0) {
            case 'P': pidHeight.setKp(value); break;
            case 'I': pidHeight.setKi(value); break;
            case 'D': pidHeight.setKd(value); break;
            default: {
                char buf[40];
                snprintf(buf, sizeof(buf), "[BT] Unbekannt: %s", cmd.c_str());
                sendLine(buf);
            }
        }
    } else {
        char buf[40];
        snprintf(buf, sizeof(buf), "[BT] Unbekannt: %s", cmd.c_str());
        sendLine(buf);
    }
}

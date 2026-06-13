#include "comm/BluetoothComm.h"
#include "myLogger.h"
#include "config.h"
#include "pins.h"

// Echo-Fenster nach dem Senden verlängern (HC-05 spiegelt TX zurück an RX)
static void _extendEcho(uint32_t &echoUntilMs, uint16_t msgLen) {
    uint32_t windowMs = (msgLen * 2u) + 30u; // 9600 Baud: ~1ms/Byte + Puffer
    uint32_t deadline = millis() + windowMs;
    if (deadline > echoUntilMs) echoUntilMs = deadline;
}

void BluetoothComm::send(const char* msg) {
    BT_UART.print(msg);
    _extendEcho(_echoUntilMs, strlen(msg));
}

void BluetoothComm::sendLine(const char* msg) {
    BT_UART.println(msg);
    _extendEcho(_echoUntilMs, strlen(msg) + 2); // +2 für \r\n
}

void BluetoothComm::begin() {
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    BT_UART.begin(BT_BAUD);
    sendLine("[BT] Drohne bereit");
    sendLine("[BT] Befehle: P=x.x | I=x.x | D=x.x | RP=x.x | ?");
    LOG("[BT] Bluetooth bereit");
}

KeyEvent BluetoothComm::getKey() {
    _command = "";

    while (BT_UART.available()) {
        uint8_t c = BT_UART.read();

        if (millis() < _echoUntilMs) continue; // Echo unterdrücken

        if (c == '\n' || c == '\r') {
            KeyEvent ev = _flushBuffer();
            if (ev != KeyEvent::NONE) return ev;
        } else {
            _buffer += (char)c;
            _lastCharMs = millis();
            if (_buffer.length() > 100) _buffer = "";
        }
    }

    // Kein Newline nach 150 ms → trotzdem auswerten
    if (_buffer.length() > 0 && (millis() - _lastCharMs >= 150)) {
        KeyEvent ev = _flushBuffer();
        if (ev != KeyEvent::NONE) return ev;
    }

    return KeyEvent::NONE;
}

String BluetoothComm::getCommand() {
    return _command;
}

KeyEvent BluetoothComm::_flushBuffer() {
    if (_buffer.length() == 0) return KeyEvent::NONE;

    if (_buffer.length() == 1) {
        char ch = toupper(_buffer[0]);
        _buffer = "";
        _lastCharMs = 0;
        switch (ch) {
            case 'A': return KeyEvent::KEY_A;
            case 'S': return KeyEvent::KEY_S;
            case 'H': return KeyEvent::KEY_H;
            case 'R': return KeyEvent::KEY_R;
            case 'L': return KeyEvent::KEY_L;
            case '+': return KeyEvent::ARROW_UP;
            case '-': return KeyEvent::ARROW_DOWN;
            default:  return KeyEvent::NONE;
        }
    }

    _command = _buffer;
    _buffer = "";
    _lastCharMs = 0;
    return KeyEvent::NONE;
}

void BluetoothComm::processCommand(const String &cmd,
                                   PIDController &pidHeight,
                                   PIDController &pidRoll,
                                   PIDController &pidPitch,
                                   Settings &settings) {
    if (cmd.length() == 0) return;

    if (cmd == "?") {
        sendLine("[BT] === Hoehe ===");
        char buf[32];
        snprintf(buf, sizeof(buf), "[BT] Kp=%.4f", pidHeight.getKp()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Ki=%.4f", pidHeight.getKi()); sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Kd=%.4f", pidHeight.getKd()); sendLine(buf);
        sendLine("[BT] === Roll ===");
        snprintf(buf, sizeof(buf), "[BT] Kp=%.4f", pidRoll.getKp());   sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Ki=%.4f", pidRoll.getKi());   sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Kd=%.4f", pidRoll.getKd());   sendLine(buf);
        sendLine("[BT] === Pitch ===");
        snprintf(buf, sizeof(buf), "[BT] Kp=%.4f", pidPitch.getKp());  sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Ki=%.4f", pidPitch.getKi());  sendLine(buf);
        snprintf(buf, sizeof(buf), "[BT] Kd=%.4f", pidPitch.getKd());  sendLine(buf);
        return;
    }

    char p0 = toupper(cmd[0]);
    char p1 = cmd.length() > 1 ? toupper(cmd[1]) : 0;

    if (p0 == 'R' && p1 == 'P' && cmd[2] == '=') {
        pidRoll.setKp(cmd.substring(3).toFloat());
    } else if (p0 == 'R' && p1 == 'I' && cmd[2] == '=') {
        pidRoll.setKi(cmd.substring(3).toFloat());
    } else if (p0 == 'R' && p1 == 'D' && cmd[2] == '=') {
        pidRoll.setKd(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'P' && cmd[2] == '=') {
        pidPitch.setKp(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'I' && cmd[2] == '=') {
        pidPitch.setKi(cmd.substring(3).toFloat());
    } else if (p0 == 'P' && p1 == 'D' && cmd[2] == '=') {
        pidPitch.setKd(cmd.substring(3).toFloat());
    } else if (cmd.length() >= 3 && p1 == '=') {
        float value = cmd.substring(2).toFloat();
        switch (p0) {
            case 'P': pidHeight.setKp(value); break;
            case 'I': pidHeight.setKi(value); break;
            case 'D': pidHeight.setKd(value); break;
        }
    }
}

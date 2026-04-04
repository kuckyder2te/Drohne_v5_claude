#include "comm/BluetoothConfig.h"
#include "config.h"

void BluetoothConfig::begin() {
    Serial1.setTX(PIN_BT_TX);   // ← wichtig für Pico!
    Serial1.setRX(PIN_BT_RX);   // ← wichtig für Pico!
    BT_UART.begin(BT_BAUD);
    BT_UART.println("[BT] Drohne bereit");
    BT_UART.println("[BT] Befehle: P=x.x | I=x.x | D=x.x | ?");
    Serial.println("[BT] Bluetooth bereit");
}

void BluetoothConfig::update(PIDController& pid) {
    while (BT_UART.available()) {
        char c = BT_UART.read();

        if (c == '\n' || c == '\r') {
            if (_buffer.length() > 0) {
                _processCommand(_buffer, pid);
                _buffer = "";
            }
        } else {
            _buffer += c;
            // Puffer-Überlauf verhindern
            if (_buffer.length() > 20) _buffer = "";
        }
    }
}

void BluetoothConfig::_processCommand(const String& cmd, PIDController& pid) {
    Serial.print("[BT] Empfangen: ");
    Serial.println(cmd);

    if (cmd == "?") {
        _printValues(pid);
        return;
    }

    if (cmd.length() < 3 || cmd[1] != '=') {
        BT_UART.println("[BT] Fehler: Format P=x.x");
        return;
    }

    char param  = toupper(cmd[0]);
    float value = cmd.substring(2).toFloat();

    switch (param) {
        case 'P':
            pid.setKp(value);
            BT_UART.print("[BT] Kp=");
            BT_UART.println(value, 4);
            break;
        case 'I':
            pid.setKi(value);
            BT_UART.print("[BT] Ki=");
            BT_UART.println(value, 4);
            break;
        case 'D':
            pid.setKd(value);
            BT_UART.print("[BT] Kd=");
            BT_UART.println(value, 4);
            break;
        default:
            BT_UART.println("[BT] Fehler: Unbekannter Parameter");
            break;
    }
}

void BluetoothConfig::_printValues(PIDController& pid) {
    BT_UART.print("[BT] Kp="); BT_UART.println(pid.getKp(), 4);
    BT_UART.print("[BT] Ki="); BT_UART.println(pid.getKi(), 4);
    BT_UART.print("[BT] Kd="); BT_UART.println(pid.getKd(), 4);
}
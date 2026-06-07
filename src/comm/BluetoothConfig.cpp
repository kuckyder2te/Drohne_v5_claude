#include "comm/BluetoothConfig.h"
#include "myLogger.h"
#include "config.h"
#include "pins.h"

void BluetoothConfig::begin() {
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    BT_UART.begin(BT_BAUD);
    BT_UART.println("[BT] Drohne bereit");
    BT_UART.println("[BT] Befehle: P=x.x | I=x.x | D=x.x | S=save | R=reset | ?");
    LOG("[BT] Bluetooth bereit");
}

// Wird von main.cpp aufgerufen mit String von keyboard.getBTCommand()
void BluetoothConfig::processCommand(const String &cmd,
                                      PIDController &pidHeight,
                                      PIDController &pidRoll,
                                      PIDController &pidPitch,
                                      Settings &settings) {
    if (cmd.length() > 0) {
        _processCommand(cmd, pidHeight, pidRoll, pidPitch, settings);
    }
}

// update() liest NICHT mehr BT_UART — keyboard.getKey() macht das!
void BluetoothConfig::update(PIDController &pidHeight,
                              PIDController &pidRoll,
                              PIDController &pidPitch,
                              Settings &settings) {
    // Leer — wird nicht mehr genutzt
    // Bleibt für Kompatibilität
}

void BluetoothConfig::_processCommand(const String &cmd,
                                       PIDController &pidHeight,
                                       PIDController &pidRoll,
                                       PIDController &pidPitch,
                                       Settings &settings) {
    if (cmd == "?") {
        BT_UART.println("[BT] === Hoehe ===");
        BT_UART.print("[BT] Kp="); BT_UART.println(pidHeight.getKp(), 4);
        BT_UART.print("[BT] Ki="); BT_UART.println(pidHeight.getKi(), 4);
        BT_UART.print("[BT] Kd="); BT_UART.println(pidHeight.getKd(), 4);
        BT_UART.println("[BT] === Roll ===");
        BT_UART.print("[BT] Kp="); BT_UART.println(pidRoll.getKp(), 4);
        BT_UART.println("[BT] === Pitch ===");
        BT_UART.print("[BT] Kp="); BT_UART.println(pidPitch.getKp(), 4);
        return;
    }

    char param0 = toupper(cmd[0]);
    char param1 = cmd.length() > 1 ? toupper(cmd[1]) : 0;

    if (param0 == 'R' && param1 == 'P') {
        pidRoll.setKp(cmd.substring(3).toFloat());
    } else if (param0 == 'R' && param1 == 'I') {
        pidRoll.setKi(cmd.substring(3).toFloat());
    } else if (param0 == 'R' && param1 == 'D') {
        pidRoll.setKd(cmd.substring(3).toFloat());
    } else if (param0 == 'P' && param1 == 'P') {
        pidPitch.setKp(cmd.substring(3).toFloat());
    } else if (param0 == 'P' && param1 == 'I') {
        pidPitch.setKi(cmd.substring(3).toFloat());
    } else if (param0 == 'P' && param1 == 'D') {
        pidPitch.setKd(cmd.substring(3).toFloat());
    } else if (param0 == 'S') {
        settings.save(pidHeight.getKp(), pidHeight.getKi(), pidHeight.getKd());
        LOG("[BT] Gespeichert!");
    } else if (param0 == 'R') {
        settings.reset();
        LOG("[BT] Reset!");
    } else if (cmd.length() >= 3 && cmd[1] == '=') {
        float value = cmd.substring(2).toFloat();
        switch (param0) {
            case 'P': pidHeight.setKp(value); break;
            case 'I': pidHeight.setKi(value); break;
            case 'D': pidHeight.setKd(value); break;
        }
    }
}

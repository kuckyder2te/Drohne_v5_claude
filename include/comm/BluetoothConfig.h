#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "storage/Settings.h"
#include "comm/KeyboardInput.h"

class BluetoothConfig {
public:
    void begin();

    // Verarbeitet einen String Befehl (von keyboard.getBTCommand())
    void processCommand(const String &cmd,
                        PIDController &pidHeight,
                        PIDController &pidRoll,
                        PIDController &pidPitch,
                        Settings &settings);

    // Alte update() Methode — liest nicht mehr BT_UART!
    void update(PIDController &pidHeight,
                PIDController &pidRoll,
                PIDController &pidPitch,
                Settings &settings);

private:
    void _processCommand(const String &cmd,
                         PIDController &pidHeight,
                         PIDController &pidRoll,
                         PIDController &pidPitch,
                         Settings &settings);
};

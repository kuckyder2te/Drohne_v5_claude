#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "storage/Settings.h"
#include "pins.h"

class BluetoothConfig
{
public:
    void begin();
    void update(PIDController &pidHeight,
                PIDController &pidRoll,
                PIDController &pidPitch,
                Settings &settings);

private:
    String _buffer = "";
    void _processCommand(const String &cmd,
                         PIDController &pidHeight,
                         PIDController &pidRoll,
                         PIDController &pidPitch,
                         Settings &settings);
    void _printValues(PIDController &pid);
};
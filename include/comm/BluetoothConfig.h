#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "storage/Settings.h"
#include "pins.h"

class BluetoothConfig {
public:
    void begin();
    void update(PIDController& pid, Settings& settings);

private:
    String _buffer = "";
    void _processCommand(const String& cmd, PIDController& pid, Settings& settings);
    void _printValues(PIDController& pid);
};
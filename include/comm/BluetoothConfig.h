#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "pins.h"

class BluetoothConfig {
public:
    void begin();
    void update(PIDController& pid);  // Aufruf im loop()

private:
    String _buffer = "";
    void _processCommand(const String& cmd, PIDController& pid);
    void _printValues(PIDController& pid);
};
#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "storage/Settings.h"
#include "comm/KeyboardInput.h"
#include "pins.h"

class BluetoothConfig
{
public:
    void begin();
    void update(PIDController &pidHeight,
                PIDController &pidRoll,
                PIDController &pidPitch,
                Settings &settings);

    // Inline in .h — kein Eintrag in .cpp nötig
    KeyEvent getPendingKey()
    {
        KeyEvent k = _pendingKey;
        _pendingKey = KeyEvent::NONE;
        return k;
    }

private:
    String _buffer = "";
    void _processCommand(const String &cmd,
                         PIDController &pidHeight,
                         PIDController &pidRoll,
                         PIDController &pidPitch,
                         Settings &settings);
    void _printValues(PIDController &pid);

    KeyEvent _pendingKey = KeyEvent::NONE;
};
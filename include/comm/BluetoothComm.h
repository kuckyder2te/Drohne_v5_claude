#pragma once

#include <Arduino.h>
#include "comm/SerialInput.h"
#include "control/PIDController.h"
#include "storage/Settings.h"

class BluetoothComm {
public:
    void begin();
    KeyEvent getKey();
    String getCommand();
    void processCommand(const String &cmd,
                        PIDController &pidHeight,
                        PIDController &pidRoll,
                        PIDController &pidPitch,
                        Settings &settings);

    void send(const char* msg);
    void sendLine(const char* msg);
    void send(const String& msg)     { send(msg.c_str()); }
    void sendLine(const String& msg) { sendLine(msg.c_str()); }

private:
    String   _buffer;
    String   _command;
    uint32_t _lastCharMs  = 0;
    uint32_t _echoUntilMs = 0;

    KeyEvent _flushBuffer();
};

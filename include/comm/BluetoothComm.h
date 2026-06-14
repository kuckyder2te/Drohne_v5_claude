#pragma once

#include <Arduino.h>
#include "control/PIDController.h"
#include "storage/Settings.h"

enum class KeyEvent {
    NONE,
    ARROW_UP,
    ARROW_DOWN,
    KEY_A,
    KEY_S,
    KEY_H,
    KEY_R,
    KEY_L,
};

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
    uint32_t _echoUntilMs = 0;
    uint32_t _lastCharMs  = 0;
};

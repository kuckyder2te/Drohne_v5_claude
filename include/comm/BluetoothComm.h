#pragma once

#include <Arduino.h>
#include "comm/KeyEvent.h"
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
    void flushEcho();

private:
    String   _buffer;
    String   _command;
    uint32_t _lastCharMs = 0;

    // Echo-Filter: gesendete Bytes in Ringpuffer, getKey() verwirft Treffer
    char    _sentBuf[256];
    uint8_t _sentHead = 0;
    uint8_t _sentTail = 0;
    void _pushSent(char c);
    bool _popSent(char c);
};

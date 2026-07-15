#pragma once

#include <Arduino.h>
#include "comm/KeyEvent.h"
#include "control/PIDController.h"
#include "storage/Settings.h"

// Transport-unabhaengiger Key-/Kommando-Parser. Nimmt beliebiges Stream-Objekt
// entgegen (HardwareSerial fuer BT-UART oder USB-Serial heute, z.B. ein TCP
// Client waere spaeter ebenso moeglich, da Stream deren gemeinsame Basisklasse ist).
class CommChannel {
public:
    explicit CommChannel(Stream &stream) : _stream(stream) {}

    KeyEvent getKey();
    String   getCommand() const { return _command; }
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
    Stream  &_stream;
    String   _buffer;
    String   _command;
    uint8_t  _escSt = 0;
    uint32_t _lastCharMs = 0;

    KeyEvent resolveSingleChar(char ch);
};

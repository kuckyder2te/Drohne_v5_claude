#pragma once

#include <Arduino.h>
#include "comm/KeyEvent.h"

class SerialInput {
public:
    KeyEvent getKey();
    String getCommand() const { return _cmd; }

private:
    uint8_t _escSt = 0;
    String  _buf;
    String  _cmd;
};

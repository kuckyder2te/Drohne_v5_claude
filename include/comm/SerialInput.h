#pragma once

#include <Arduino.h>

enum class KeyEvent {
    NONE,
    ARROW_UP,
    ARROW_DOWN,
    KEY_A,
    KEY_S,
    KEY_H,
    KEY_R,
    KEY_L
};

class SerialInput {
public:
    void begin();
    KeyEvent getKey();

private:
    uint8_t _escState = 0;
    KeyEvent _parseChar(uint8_t c);
};

#pragma once

#include <Arduino.h>
#include "config.h"

enum class KeyEvent {
    NONE,
    ARROW_UP,
    ARROW_DOWN,
    KEY_A,
    KEY_S,
    KEY_H,
    KEY_R
};

class KeyboardInput {
public:
    void begin();
    KeyEvent getKey();
    String getBTCommand();  // ← neu: gibt gepufferten BT String zurück

private:
    uint8_t _escState = 0;
    String  _btBuffer = "";     // BT String Puffer
    String  _btCommand = "";    // fertiger BT Befehl

    KeyEvent _parseChar(uint8_t c, bool fromBT);
};

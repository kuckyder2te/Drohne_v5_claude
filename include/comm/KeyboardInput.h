#pragma once

#include <Arduino.h>

// Rückgabewerte für getKey()
enum class KeyEvent {
    NONE,
    ARROW_UP,
    ARROW_DOWN,
    KEY_S,
    KEY_H,
    KEY_R,
    KEY_A,    // ← neu: ARM
};

class KeyboardInput {
public:
    void begin();
    KeyEvent getKey();  // Aufruf im loop(), non-blocking

private:
    // ANSI Escape-Sequenz Parser
    uint8_t _escState = 0;  // 0=idle, 1=got ESC, 2=got [
};
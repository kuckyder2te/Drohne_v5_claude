#pragma once

#include <Arduino.h>

class Battery {
public:
    void begin();
    void update();

    float getVoltage() const { return _voltage; }
    bool isWarning()  const { return _voltage < BATTERY_WARN_V; }
    bool isCritical() const { return _voltage < BATTERY_CRIT_V; }

private:
    static constexpr float    BATTERY_WARN_V  = 10.5f;
    static constexpr float    BATTERY_CRIT_V  = 10.0f;
    static constexpr float    BATTERY_FACTOR  = 6.0f;   // (100k+20k)/20k
    static constexpr float    BATTERY_VREF    = 3.3f;
    static constexpr float    BATTERY_ADC_MAX = 4095.0f;

    float    _voltage     = 12.6f;

    // Buzzer non-blocking
    uint32_t _lastCheck   = 0;   // letzter Batterie-Check
    uint32_t _beepTimer   = 0;   // Buzzer Timer
    int      _beepCount   = 0;   // verbleibende Beeps
    bool     _buzzerOn    = false;

    void _startBeep(int count);
    void _updateBuzzer();
};

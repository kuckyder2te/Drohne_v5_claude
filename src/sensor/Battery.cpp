#include "sensor/Battery.h"
#include "myLogger.h"
#include "pins.h"

void Battery::begin()
{
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    analogReadResolution(12);
    LOG("[BAT] Batterie-Ueberwachung gestartet");
}

void Battery::update()
{
    uint32_t now = millis();

    _updateBuzzer();

    if (now - _lastCheck < 1000)
        return;
    _lastCheck = now;

    // Spannung messen
    int raw = 0;
    for (int i = 0; i < 5; i++)
    {
        raw += analogRead(BATTERY);
    }
    raw /= 5;
    _voltage = (raw / BATTERY_ADC_MAX) * BATTERY_VREF * BATTERY_FACTOR;

    // Warnstufen — eigener Timer!
    if (_voltage < BATTERY_CRIT_V)
    {
        if (_beepCount == 0)
        { // ← nur wenn kein Beep läuft
            _startBeep(3);
        }
    }
    else if (_voltage < BATTERY_WARN_V)
    {
        if (_beepCount == 0)
        { // ← nur wenn kein Beep läuft
            _startBeep(1);
        }
    }
}

void Battery::_startBeep(int count)
{
    _beepCount = count * 2; // AN + AUS pro Beep
    _beepTimer = millis();
    _buzzerOn = false;
}

void Battery::_updateBuzzer()
{

    if (_beepCount <= 0)
    {
        digitalWrite(BUZZER, LOW);
        return;
    }

    uint32_t now = millis();
    if (now - _beepTimer < 100)
        return; // 100ms pro Schritt
    _beepTimer = now;

    _buzzerOn = !_buzzerOn;
    digitalWrite(BUZZER, _buzzerOn ? HIGH : LOW);
    _beepCount--;
}

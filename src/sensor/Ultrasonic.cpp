#include "sensor/Ultrasonic.h"
#include "myLogger.h"
#include "pins.h"

void Ultrasonic::begin()
{
    pinMode(PIN_ULTRASONIC_TRIG1, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO1, INPUT);
    digitalWrite(PIN_ULTRASONIC_TRIG1, LOW);

    // pinMode(PIN_ULTRASONIC_TRIG2, OUTPUT);
    //pinMode(PIN_ULTRASONIC_ECHO2, INPUT);
    // digitalWrite(PIN_ULTRASONIC_TRIG2, LOW);

    delay(100);
    LOG("[ULTRA] HC-SR04 bereit");
}

void Ultrasonic::update()
{
    _trigger();
    float d1 = _measureCm(PIN_ULTRASONIC_ECHO1);

    bool valid1 = (d1 >= ULTRASONIC_MIN_CM && d1 <= ULTRASONIC_MAX_CM);

    if (valid1)
    {
        _valid = true;
        _altitudeCm = _applyFilter(d1);
    }
    else
    {
        _valid = false;
    }
}

void Ultrasonic::_trigger()
{
    digitalWrite(PIN_ULTRASONIC_TRIG1, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG1, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG1, LOW);
}

float Ultrasonic::_measureCm(uint8_t echoPin)
{
    // Echo Zeit messen (Timeout 25ms = ~4m)
    uint32_t duration = pulseIn(echoPin, HIGH, 25000);
    if (duration == 0)
        return -1.0f; // Timeout

    // Zeit → Distanz (Schallgeschwindigkeit 343 m/s)
    // duration in µs → cm: duration * 0.0343 / 2
    return (duration * 0.0343f) / 2.0f;
}

float Ultrasonic::_applyFilter(float newValue)
{
    _filterBuf[_filterIdx] = newValue;
    _filterIdx = (_filterIdx + 1) % ULTRASONIC_FILTER_SIZE;
    if (_filterIdx == 0)
        _filterFull = true;

    uint8_t count = _filterFull ? ULTRASONIC_FILTER_SIZE : _filterIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
        sum += _filterBuf[i];
    return sum / count;
}

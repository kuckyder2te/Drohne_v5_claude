#include "Ultrasonic.h"
#include "myLogger.h"
#include "pins.h"


void Ultrasonic::begin()
{
    _ch[0].trigPin = PIN_ULTRASONIC_TRIG1;
    _ch[0].echoPin = PIN_ULTRASONIC_ECHO1;
    _ch[1].trigPin = PIN_ULTRASONIC_TRIG2;
    _ch[1].echoPin = PIN_ULTRASONIC_ECHO2;

    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
    {
        pinMode(_ch[i].trigPin, OUTPUT);
        pinMode(_ch[i].echoPin, INPUT);
        digitalWrite(_ch[i].trigPin, LOW);
    }
    delay(100);

    // Ein Probe-Ping je Sensor. begin() laeuft vor dem Start von Kern 1 und
    // darf blockieren - dafuer faellt ein nicht angeschlossener Sensor sofort
    // im Bootlog auf, statt erst im Flug als stiller Ausfall.
    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
    {
        _trigger(_ch[i].trigPin);
        float d = _measureCm(_ch[i].echoPin);
        // Klammern sind Pflicht: LOGGER_*_FMT expandiert zu ZWEI Anweisungen
        // (sprintf + Logger-Aufruf), ohne do/while-Huelle.
        if (d >= ULTRASONIC_MIN_CM && d <= ULTRASONIC_MAX_CM) {
            LOGGER_NOTICE_FMT("[ULTRA] S%u (TRIG GP%u / ECHO GP%u): %.1f cm",
                              (unsigned)(i + 1), (unsigned)_ch[i].trigPin,
                              (unsigned)_ch[i].echoPin, d);
        } else {
            LOGGER_NOTICE_FMT("[ULTRA] S%u (TRIG GP%u / ECHO GP%u): kein Echo - Verkabelung pruefen!",
                              (unsigned)(i + 1), (unsigned)_ch[i].trigPin,
                              (unsigned)_ch[i].echoPin);
        }
        delay(ULTRASONIC_MIN_PING_MS);
    }

    LOGGER_NOTICE("[ULTRA] 2x HC-SR04 bereit (Wechsel je update, Fusion = Minimum)");
}

void Ultrasonic::update()
{
    // Mindestabstand zwischen zwei Pings. Ersetzt das fruehere delay(30) am
    // Ende dieser Funktion: dort hat es Kern 0 blockiert, hier kostet es nichts
    // und wirkt auch dann, wenn der Aufrufer keine eigene Kadenz hat.
    uint32_t now = millis();
    if (_lastTriggerMs != 0 && (now - _lastTriggerMs) < ULTRASONIC_MIN_PING_MS)
        return;
    _lastTriggerMs = now;

    Chan &c = _ch[_current];

    _trigger(c.trigPin);
    float d = _measureCm(c.echoPin);

    if (d >= ULTRASONIC_MIN_CM && d <= ULTRASONIC_MAX_CM)
    {
        c.altitudeCm  = _applyFilter(c, d);
        c.lastValidMs = millis();
        if (c.lastValidMs == 0) c.lastValidMs = 1;  // 0 heisst "nie gemessen"
    }
    // Sonst: nichts tun. Filter und letzter Wert bleiben absichtlich stehen -
    // ein einzelner Aussetzer soll den Kanal nicht sofort aus der Fusion
    // werfen, sonst spraenge die Hoehe auf den anderen Sensor und zurueck.
    // Ueber lastValidMs faellt er nach ULTRASONIC_STALE_MS von selbst raus.

    _current = (_current + 1) % ULTRASONIC_COUNT;

    _fuse();
}

// Minimum ueber alle frischen Kanaele.
void Ultrasonic::_fuse()
{
    float best  = 0.0f;
    bool  found = false;

    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
    {
        if (!isValid(i))
            continue;
        if (!found || _ch[i].altitudeCm < best)
        {
            best  = _ch[i].altitudeCm;
            found = true;
        }
    }

    // Ist kein Kanal mehr frisch, bleibt _altitudeCm bewusst STEHEN - genau wie
    // im Einzelsensor-Stand. FlightController::updateControlLoop() liest ohne
    // BARO_ENABLED getAltitudeCm() ohne isValid()-Pruefung und verlaesst sich
    // darauf, dass der letzte gute Wert einen Aussetzer ueberbrueckt.
    if (found)
        _altitudeCm = best;
}

float Ultrasonic::getAltitudeCm(uint8_t i) const
{
    return (i < ULTRASONIC_COUNT) ? _ch[i].altitudeCm : 0.0f;
}

bool Ultrasonic::isValid(uint8_t i) const
{
    if (i >= ULTRASONIC_COUNT)
        return false;
    const Chan &c = _ch[i];
    if (c.lastValidMs == 0)
        return false;   // noch nie einen gueltigen Wert geliefert
    return (millis() - c.lastValidMs) <= ULTRASONIC_STALE_MS;
}

// Live geprueft statt in _fuse() zwischengespeichert: ein Kanal kann auch
// zwischen zwei update()-Aufrufen ueberaltern.
bool Ultrasonic::isValid() const
{
    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
        if (isValid(i))
            return true;
    return false;
}

float Ultrasonic::getSpreadCm() const
{
    if (!isValid(0) || !isValid(1))
        return -1.0f;
    return fabsf(_ch[0].altitudeCm - _ch[1].altitudeCm);
}

void Ultrasonic::_trigger(uint8_t trigPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

float Ultrasonic::_measureCm(uint8_t echoPin)
{
    // Echo Zeit messen (Timeout 25ms = ~4m).
    // 4 m sind HIN UND ZURUECK 8 m, bei 343 m/s also 23,3 ms - nicht 11,6 ms.
    uint32_t duration = pulseIn(echoPin, HIGH, 25000);
    if (duration == 0)
        return -1.0f; // Timeout

    // Zeit → Distanz (Schallgeschwindigkeit 343 m/s)
    // duration in µs → cm: duration * 0.0343 / 2
    return (duration * 0.0343f) / 2.0f;
}

float Ultrasonic::_applyFilter(Chan &c, float newValue)
{
    c.filterBuf[c.filterIdx] = newValue;
    c.filterIdx = (c.filterIdx + 1) % ULTRASONIC_FILTER_SIZE;
    if (c.filterIdx == 0)
        c.filterFull = true;

    uint8_t count = c.filterFull ? ULTRASONIC_FILTER_SIZE : c.filterIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
        sum += c.filterBuf[i];
    return sum / count;
}

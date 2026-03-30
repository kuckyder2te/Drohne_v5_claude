#include "sensor/Barometer.h"
#include "pins.h"

bool Barometer::begin() {
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!_ms5611.begin()) {
        Serial.println("[BARO] ERROR: MS5611 nicht gefunden!");
        return false;
    }

    Serial.println("[BARO] MS5611 gefunden");

    // Referenzdruck aus 10 Messungen mitteln
    delay(100);
    calibrate();
    return true;
}

void Barometer::calibrate() {
    Serial.println("[BARO] Kalibrierung läuft...");
    float sum = 0.0f;
    for (int i = 0; i < 10; i++) {
        _ms5611.read();
        sum += _ms5611.getPressure();
        delay(50);
    }
    _refPressure = sum / 10.0f;
    Serial.print("[BARO] Referenzdruck: ");
    Serial.print(_refPressure);
    Serial.println(" Pa");
}

void Barometer::update() {
    int result = _ms5611.read();
    if (result != MS5611_READ_OK) {
        Serial.println("[BARO] Lesefehler!");
        return;
    }

    _pressure    = _ms5611.getPressure();
    _temperature = _ms5611.getTemperature();

    // Barometrische Höhenformel → Ergebnis in cm
    // h = 44330 * (1 - (p / p0) ^ (1/5.255))
    float ratio   = _pressure / _refPressure;
    float altitudeM = 44330.0f * (1.0f - pow(ratio, 0.1902949f));
    _altitudeCm   = altitudeM * 100.0f;
}
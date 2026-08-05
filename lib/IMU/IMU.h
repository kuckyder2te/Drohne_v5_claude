#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>

class IMU
{
public:
    bool begin(bool initWire = false);

    // Ohne Argument wie bisher: dt wird intern aus micros() gebildet.
    bool update();
    // Mit vorgegebenem Zeitstempel - vom Lageregelkreis auf Kern 1 benutzt,
    // der micros() ohnehin schon fuer seine Taktung liest.
    bool update(uint32_t nowUs);

    void calibrate();

    float getRoll()  const { return _roll; }
    float getPitch() const { return _pitch; }
    float getAccZ()  const { return _accelZ; }

    // Rohe Drehraten in Grad/s. Sie sind die Ableitung des Winkels und damit
    // die brauchbare Quelle fuer den D-Anteil: der Differenzenquotient ueber
    // den gefilterten Winkel ist bei 400 Hz zu verrauscht.
    float getGyroRoll()  const { return _gyroRoll; }
    float getGyroPitch() const { return _gyroPitch; }

    uint32_t getLastUpdateUs() const { return _lastUpdateUs; }
    bool     isReady()         const { return _ready; }

    // Zeitkonstante des Komplementaerfilters in Sekunden. alpha wird daraus
    // je Zyklus als tau/(tau+dt) gebildet - ein fester alpha-Wert waere nur
    // fuer genau eine Zykluszeit richtig und verstellt sich still, sobald
    // die Schleife schneller oder langsamer laeuft.
    void  setTau(float tauSeconds) { _tau = tauSeconds; }
    float getTau() const { return _tau; }

private:
    // AD0 liegt an GND, meldet sich auf diesem Board aber unter 0x69 (nicht 0x68)
    ICM20948_WE _imu{0x69};

    float _roll      = 0.0f;
    float _pitch     = 0.0f;
    float _accelZ    = 0.0f;
    float _gyroRoll  = 0.0f;
    float _gyroPitch = 0.0f;
    bool  _ready     = false;

    float    _tau          = 0.5f;
    uint32_t _lastUpdateUs = 0;
    bool     _firstUpdate  = true;

    void _apply(uint32_t nowUs);
};

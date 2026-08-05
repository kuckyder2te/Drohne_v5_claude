#include "IMU.h"
#include "myLogger.h"
#include "pins.h"
#include "config.h"

bool IMU::begin(bool initWire) {
    if (initWire) {
        // Bus-Recovery: 9 SCL-Pulse befreien haengenden SDA
        pinMode(PIN_SDA, OUTPUT);
        pinMode(PIN_SCL, OUTPUT);
        digitalWrite(PIN_SDA, HIGH);
        for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
            digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
        }
        digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
        digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);

        Wire.setSDA(PIN_SDA);
        Wire.setSCL(PIN_SCL);
        Wire.begin();
        Wire.setClock(400000);
        delay(100);
    }

    if (!_imu.init()) {
        LOGGER_NOTICE("[IMU] ERROR: ICM-20948 nicht gefunden!");
        return false;
    }

    LOGGER_NOTICE("[IMU] Kalibrierung - bitte ruhig halten...");
    _imu.autoOffsets();

    // Messbereich Gyro 500 statt 250 dps: bei der Relais-Anregung des
    // Autotunings treten kurzzeitig hohe Drehraten auf, und ein saettigender
    // Gyro wuerde die Periodenmessung verfaelschen.
    _imu.setGyrRange(ICM20948_GYRO_RANGE_500);
    _imu.setAccRange(ICM20948_ACC_RANGE_2G);

    // DLPF_6 waren 5,7 Hz Bandbreite auf BEIDEN Sensoren - mehr als 30 ms
    // Gruppenlaufzeit. Das war der eigentliche Grund, warum die Lage dem
    // Regler hinterherlief. Bei ATTITUDE_RATE_HZ (Nyquist 200 Hz):
    //   Gyro  DLPF_3 = 51,2 Hz  -> ~4 ms Laufzeit, Propellerblattfolge
    //                             (>250 Hz) bleibt sicher unterdrueckt
    //   Accel DLPF_4 = 23,9 Hz  -> stuetzt den Filter nur langsam, 24 Hz
    //                             genuegt und daempft Vibration deutlich besser
    // Wirkt der D-Anteil traege, ist Gyro DLPF_2 (119,5 Hz) der naechste
    // Schritt - dann aber im Recorder-Dump auf Vibration gegenpruefen.
    _imu.setGyrDLPF(ICM20948_DLPF_3);
    _imu.setAccDLPF(ICM20948_DLPF_4);

    // Volle Ausgaberate (1125 Hz), damit jedes Lesen frische Daten bekommt.
    _imu.setGyrSampleRateDivider(0);
    _imu.setAccSampleRateDivider(0);

    _tau  = IMU_TAU_S;
    _ready = true;
    LOGGER_NOTICE("[IMU] ICM-20948 bereit");
    return true;
}

void IMU::calibrate() {
    LOGGER_NOTICE("[IMU] Rekalibrierung - bitte ruhig halten...");
    _imu.autoOffsets();
    LOGGER_NOTICE("[IMU] Kalibrierung abgeschlossen");
}

bool IMU::update() {
    return update(micros());
}

bool IMU::update(uint32_t nowUs) {
    if (!_ready) return false;

    _imu.readSensor();
    _apply(nowUs);
    return true;
}

void IMU::_apply(uint32_t nowUs) {
    xyzFloat gVal;
    xyzFloat gyr;
    _imu.getGValues(&gVal);   // in g
    _imu.getGyrValues(&gyr);  // in deg/s

    _accelZ    = gVal.z * 9.81f;
    _gyroRoll  = gyr.x;
    _gyroPitch = gyr.y;

    // Vorzeichenlose Differenz -> ueberlaufsicher (micros() laeuft alle
    // 71,6 min ueber).
    float dt = _firstUpdate ? (1.0f / ATTITUDE_RATE_HZ)
                            : (uint32_t)(nowUs - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = nowUs;
    _firstUpdate  = false;

    // Aussetzer nicht in den Filter durchschlagen lassen: bei einem
    // ausgelassenen Zyklus waere die Gyro-Integration sonst der falschen
    // Zeitbasis gefolgt.
    if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / ATTITUDE_RATE_HZ;

    float accelRoll  = atan2f(gVal.y, gVal.z) * 180.0f / M_PI;
    float accelPitch = atan2f(-gVal.x, sqrtf(gVal.y*gVal.y + gVal.z*gVal.z)) * 180.0f / M_PI;

    // alpha aus der Zeitkonstante statt fest verdrahtet. Frueher stand hier
    // alpha=0.98; bei dem damaligen 0,1-s-Takt entsprach das einer
    // Zeitkonstante von 4,9 s, bei 2,5 ms waeren es 0,12 s. Ueber tau ist
    // das Filterverhalten von der Zykluszeit entkoppelt.
    float alpha = _tau / (_tau + dt);

    _roll  = alpha * (_roll  + gyr.x * dt) + (1.0f - alpha) * accelRoll;
    _pitch = alpha * (_pitch + gyr.y * dt) + (1.0f - alpha) * accelPitch;
}

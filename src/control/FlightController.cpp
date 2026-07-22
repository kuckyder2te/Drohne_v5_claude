#include "control/FlightController.h"

#include "myLogger.h"
#include "config.h"
#include "Barometer.h"
#include "Ultrasonic.h"
#include "IMU.h"
#include "Battery.h"
#include "storage/Settings.h"

void FlightController::begin(Settings &settings)
{
    _motors.begin();
    _pidHeight.begin();
    _pidRoll.begin();
    _pidPitch.begin();

    settings.begin();
    float kp, ki, kd;
    if (settings.load(kp, ki, kd))
    {
        _pidHeight.setKp(kp);
        _pidHeight.setKi(ki);
        _pidHeight.setKd(kd);
    }
}

void FlightController::requestArm(bool imuReady, Barometer &baro)
{
    if (_armed)
        return;

    if (!_armPending)
    {
        _armPending = true;
        _armPendingMs = millis();
        LOG("[CTRL] ARM? Nochmal 'a' druecken (3s)");
        return;
    }

    if (millis() - _armPendingMs > 3000)
    {
        _armPending = false;
        LOG("[CTRL] ARM abgebrochen (Timeout)");
        return;
    }

    _armPending = false;
    if (!imuReady)
    {
        LOG("[CTRL] ARM verweigert - IMU nicht bereit!");
        return;
    }

    LOG("[CTRL] Rekalibrierung vor ARM...");
    baro.calibrate();
    delay(500);
    _armed = true;
    _targetHeightCm = 20.0f;
    _lastPidMs = millis();
    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    LOG("[CTRL] ARM - Ziel: 20 cm");
}

void FlightController::disarm()
{
    _armed = false;
    _targetHeightCm = 0.0f;
    _motors.stop();
    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    LOG("[CTRL] DISARM - Motoren gestoppt");
}

void FlightController::recalibrate(Barometer &baro)
{
    if (_armed)
    {
        LOG("[CTRL] Rekalibrierung nur im DISARM Modus!");
        return;
    }
    baro.calibrate();
    _pidHeight.reset();
    LOG("[CTRL] Barometer rekalibriert");
}

void FlightController::adjustTargetHeight(float deltaCm)
{
    _targetHeightCm = constrain(_targetHeightCm + deltaCm, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
    LOG_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::updateArmPendingTimeout()
{
    if (_armPending && (millis() - _armPendingMs > 3000))
    {
        _armPending = false;
        LOG("[CTRL] ARM abgebrochen (Timeout)");
    }
}

void FlightController::toggleStatusLog()
{
    _statusLogEnabled = !_statusLogEnabled;
    LOG_FMT("[CTRL] Statusausgabe: %s", _statusLogEnabled ? "EIN" : "AUS");
}

void FlightController::setTargetHeightCm(float cm)
{
    _targetHeightCm = constrain(cm, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
    LOG_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::checkSafety(bool imuReady, float baroAltitudeCm)
{
    if (!_armed)
        return;

    if (!imuReady)
    {
        LOG("[SAFETY] IMU Fehler - DISARM!");
        disarm();
    }

    if (abs(baroAltitudeCm - _lastSafetyHeightCm) > 500.0f)
    {
        LOG("[SAFETY] Hoehensprung - DISARM!");
        disarm();
    }
    _lastSafetyHeightCm = baroAltitudeCm;
}

void FlightController::updateControlLoop(const Ultrasonic &ultrasonic, const Barometer &baro, const IMU &imu)
{
    if (!_armed || (millis() - _lastPidMs < PID_INTERVAL_MS))
        return;
    _lastPidMs = millis();

    bool airborne = ultrasonic.isValid() && (ultrasonic.getAltitudeCm() > LIFTOFF_HEIGHT_CM);
    _pidHeight.enableIntegral(airborne);
    _pidRoll.enableIntegral(airborne);
    _pidPitch.enableIntegral(airborne);

    float currentHeight = ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm();
    float throttle = _pidHeight.compute(_targetHeightCm, currentHeight);

    float rollCorr = _pidRoll.compute(TARGET_ROLL_DEG, imu.getRoll());
    float pitchCorr = _pidPitch.compute(TARGET_PITCH_DEG, imu.getPitch());

    _motors.mix((uint16_t)throttle, rollCorr, pitchCorr, 0.0f);
}

void FlightController::logStatus(const Battery &battery, const Barometer &baro, const Ultrasonic &ultrasonic)
{
    if (!_statusLogEnabled || (millis() - _lastPrintMs < 500))
        return;
    _lastPrintMs = millis();

    LOG_FMT("[CTRL] Ziel: %.1f cm | Ist: %.1f cm | Throttle: %.0f us | Armed: %s | Bat: %.2fV | Druck: %.2f hPa",
            _targetHeightCm,
            ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm(),
            _pidHeight.getLastThrottle(),
            _armed ? "JA" : "NEIN",
            battery.getVoltage(),
            baro.getPressure());
}

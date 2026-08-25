#include "control/FlightController.h"

#include "myLogger.h"
#include "config.h"
#include "Barometer.h"
#include "Ultrasonic.h"
#include "IMU.h"
#include "Battery.h"
#include "storage/Settings.h"

using namespace shared;

namespace
{
    void applyCoeffs(PIDController &pid, const PidCoeffs &c)
    {
        pid.setKp(c.kp);
        pid.setKi(c.ki);
        pid.setKd(c.kd);
    }
}

void FlightController::begin(Settings &settings)
{
    // begin() gibt erst MIN aus und schaltet die ESCs danach ueber den MOSFET
    // an PIN_ESC_POWER zu - sie laufen damit im Normalbetrieb hoch und bleiben
    // fuer den Rest der Laufzeit am Strom. Der Not-Aus schaltet den Strom
    // bewusst NICHT ab: stromlose ESCs im Flug heissen freier Fall, richtig
    // ist MIN auf allen vier Kanaelen (stopFast()).
    _motors.begin();
    _pidHeight.begin();
    _pidRoll.begin();
    _pidPitch.begin();

    settings.begin();
    PidCoeffs height, roll, pitch;
    if (settings.load(height, roll, pitch))
    {
        applyCoeffs(_pidHeight, height);
        applyCoeffs(_pidRoll,   roll);
        applyCoeffs(_pidPitch,  pitch);
    }

    publish();
}

// Baut den Befehlsblock fuer Kern 1 aus dem aktuellen Zustand.
void FlightController::publish()
{
    CoreCmd c;
    c.armed       = _armed;
    c.mode        = _mode;
    c.axis        = _axis;
    c.throttleUs  = _throttleUs;
    c.targetRoll  = TARGET_ROLL_DEG;
    c.targetPitch = TARGET_PITCH_DEG;
    c.motorMaxUs  = _motorMaxUs;
    c.integralOn  = _integralOn;
    c.roll        = {_pidRoll.getKp(),  _pidRoll.getKi(),  _pidRoll.getKd()};
    c.pitch       = {_pidPitch.getKp(), _pidPitch.getKi(), _pidPitch.getKd()};
    c.bench       = _bench;
    c.tune        = _tune;
    c.runId       = _runId;
    cmdWrite(c);
}

void FlightController::requestArm(bool imuReady, Barometer &baro)
{
    if (_armed)
        return;

    if (!_armPending)
    {
        _armPending = true;
        _armPendingMs = millis();
        LOGGER_NOTICE("[CTRL] ARM? Nochmal 'arm' eingeben (3s)");
        return;
    }

    if (millis() - _armPendingMs > 3000)
    {
        _armPending = false;
        LOGGER_NOTICE("[CTRL] ARM abgebrochen (Timeout)");
        return;
    }

    _armPending = false;
    if (!imuReady)
    {
        LOGGER_NOTICE("[CTRL] ARM verweigert - IMU nicht bereit!");
        return;
    }

#ifdef BARO_ENABLED
    LOGGER_NOTICE("[CTRL] Rekalibrierung vor ARM...");
    baro.calibrate();
    delay(500);
#else
    (void)baro;   // Baro ist abgeschaltet, Kern 1 haelt den I2C-Bus (config.h)
#endif

    g_estop = false;   // eine frueher gedrueckte Not-Aus-Taste entschaerfen
    _armed = true;
    _targetHeightCm = 20.0f;
    _lastPidMs = millis();
    _mode = MODE_FLIGHT;
    _integralOn = false;
    _throttleUs = ESC_MIN_US;
    _motorMaxUs = ESC_MAX_US;
    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    publish();
    LOGGER_NOTICE("[CTRL] ARM - Ziel: 20 cm");
}

void FlightController::disarm()
{
    _armed = false;
    _targetHeightCm = 0.0f;
    _mode = MODE_IDLE;
    _throttleUs = ESC_MIN_US;
    _integralOn = false;

    // Selbst abschalten, nicht auf Kern 1 warten: stopFast() sind vier
    // Registerschreibzugriffe und braucht keine Zusammenarbeit.
    _motors.stopFast();
    _motors.setMaxUs(ESC_MAX_US);

    _pidHeight.reset();
    _pidRoll.reset();
    _pidPitch.reset();
    publish();
    LOGGER_NOTICE("[CTRL] DISARM - Motoren gestoppt");
}

void FlightController::recalibrate(Barometer &baro)
{
    if (_armed)
    {
        LOGGER_NOTICE("[CTRL] Rekalibrierung nur im DISARM Modus!");
        return;
    }
#ifdef BARO_ENABLED
    baro.calibrate();
    _pidHeight.reset();
    LOGGER_NOTICE("[CTRL] Barometer rekalibriert");
#else
    (void)baro;
    LOGGER_NOTICE("[CTRL] Barometer abgeschaltet (BARO_ENABLED in config.h)");
#endif
}

void FlightController::adjustTargetHeight(float deltaCm)
{
    _targetHeightCm = constrain(_targetHeightCm + deltaCm, (float)THROTTLE_MIN_CM, (float)MAX_HEIGHT_CM);
    LOGGER_NOTICE_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::updateArmPendingTimeout()
{
    if (_armPending && (millis() - _armPendingMs > 3000))
    {
        _armPending = false;
        LOGGER_NOTICE("[CTRL] ARM abgebrochen (Timeout)");
    }
}

void FlightController::toggleStatusLog()
{
    _statusLogEnabled = !_statusLogEnabled;
    LOGGER_NOTICE_FMT("[CTRL] Statusausgabe: %s", _statusLogEnabled ? "EIN" : "AUS");
}

void FlightController::setTargetHeightCm(float cm)
{
    _targetHeightCm = constrain(cm, (float)THROTTLE_MIN_CM, (float)MAX_HEIGHT_CM);
    LOGGER_NOTICE_FMT("[CTRL] Zielhoehe: %.1f cm", _targetHeightCm);
}

void FlightController::checkSafety(bool imuReady, float altitudeCm)
{
    if (!_armed)
        return;

    if (!imuReady)
    {
        LOGGER_NOTICE("[SAFETY] IMU Fehler - DISARM!");
        disarm();
        return;
    }

    // Am Pruefstand ist die Hoehe bedeutungslos - die Wippe haelt das Geraet
    // fest, der Ultraschall sieht je nach Neigung Sprungwerte.
    if (_mode == MODE_BENCH || _mode == MODE_TUNE)
        return;

    if (abs(altitudeCm - _lastSafetyHeightCm) > 500.0f)
    {
        LOGGER_NOTICE("[SAFETY] Hoehensprung - DISARM!");
        disarm();
    }
    _lastSafetyHeightCm = altitudeCm;
}

void FlightController::updateControlLoop(const Ultrasonic &ultrasonic, const Barometer &baro)
{
    // Nur noch der Hoehenregler. Roll/Pitch rechnet Kern 1 mit
    // ATTITUDE_RATE_HZ - hier waeren sie an die ~20 Hz dieser Schleife
    // gebunden und damit viel zu langsam fuer eine Lageregelung.
    if (!_armed || _mode != MODE_FLIGHT || (millis() - _lastPidMs < PID_INTERVAL_MS))
        return;
    _lastPidMs = millis();

    bool airborne = ultrasonic.isValid() && (ultrasonic.getAltitudeCm() > LIFTOFF_HEIGHT_CM);
    _pidHeight.enableIntegral(airborne);
    _integralOn = airborne;

#ifdef BARO_ENABLED
    float currentHeight = ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm();
#else
    (void)baro;
    float currentHeight = ultrasonic.getAltitudeCm();
#endif

    _throttleUs = _pidHeight.compute(_targetHeightCm, currentHeight);
    publish();
}

void FlightController::startBench(uint8_t axis, float throttleUs, float maxUs,
                                  const BenchCfg &cfg)
{
    _mode       = MODE_BENCH;
    _axis       = axis;
    _throttleUs = throttleUs;
    _motorMaxUs = maxUs;
    _bench      = cfg;
    _integralOn = false;
    ++_runId;
    publish();
}

void FlightController::startTune(uint8_t axis, float throttleUs, float maxUs,
                                 const TuneCfg &cfg)
{
    _mode       = MODE_TUNE;
    _axis       = axis;
    _throttleUs = throttleUs;
    _motorMaxUs = maxUs;
    _tune       = cfg;
    _integralOn = false;
    ++_runId;
    publish();
}

void FlightController::stopRun()
{
    // Bewusst IDLE und nicht zurueck nach MODE_FLIGHT: auf der Wippe steht
    // das Geraet fest, der Ultraschall misst dort je nach Neigung Unsinn.
    // Ein anlaufender Hoehenregler wuerde die Motoren gegen die Halterung
    // hochfahren. Fuer den Flug muss neu gearmt werden.
    _mode       = MODE_IDLE;
    _throttleUs = ESC_MIN_US;
    _motorMaxUs = ESC_MAX_US;
    _motors.stopFast();
    publish();
}

void FlightController::logStatus(const Battery &battery, const Barometer &baro, const Ultrasonic &ultrasonic)
{
    if (!_statusLogEnabled || (millis() - _lastPrintMs < 500))
        return;
    _lastPrintMs = millis();

    CoreTlm t;
    tlmRead(t);

#ifdef BARO_ENABLED
    float alt = ultrasonic.isValid() ? ultrasonic.getAltitudeCm() : baro.getAltitudeCm();
#else
    (void)baro;
    float alt = ultrasonic.getAltitudeCm();
#endif

    LOGGER_NOTICE_FMT("[CTRL] Ziel: %.1f cm | Ist: %.1f cm | Thr: %.0f us | Armed: %s | Bat: %.2fV | R/P: %.1f/%.1f | %lu Hz",
            _targetHeightCm,
            alt,
            _pidHeight.getLastThrottle(),
            _armed ? "JA" : "NEIN",
            battery.getVoltage(),
            t.roll, t.pitch,
            (unsigned long)t.loopHz);
}

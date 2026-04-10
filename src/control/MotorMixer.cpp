#include "control/MotorMixer.h"
#include "myLogger.h"
#include "config.h"
#include "hardware/pwm.h"

// ── Hilfsfunktion: Pin als PWM konfigurieren ───────────────
static void pwm_init_pin(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    // 50 Hz: Systemtakt 125 MHz / (2500 * 1000) = 50 Hz
    pwm_set_clkdiv(slice, 125.0f); // 125 MHz / 125 = 1 MHz
    pwm_set_wrap(slice, 20000);    // 1 MHz / 20000 = 50 Hz
    pwm_set_enabled(slice, true);
}

// ── µs in PWM-Counter-Wert umrechnen ──────────────────────
static uint16_t us_to_count(uint16_t us)
{
    // Bei 1 MHz Takt = 1 Count pro µs
    return us;
}

void MotorMixer::_writePWM(uint8_t pin, uint16_t us)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, us_to_count(us));
}

void MotorMixer::begin()
{
    pwm_init_pin(PIN_MOTOR_FL);
    pwm_init_pin(PIN_MOTOR_FR);
    pwm_init_pin(PIN_MOTOR_BL);
    pwm_init_pin(PIN_MOTOR_BR);

    // Alle ESCs mit Minimalthrottle initialisieren
    stop();
    delay(2000); // ESCs kalibrieren lassen
    LOG("[MOTOR] ESC Initialisierung abgeschlossen");
}

void MotorMixer::setThrottle(uint16_t throttle_us)
{
    // Sicherheitsgrenzen einhalten
    _throttle_us = constrain(throttle_us, ESC_MIN_US, ESC_MAX_US);

    _writePWM(PIN_MOTOR_FL, _throttle_us);
    _writePWM(PIN_MOTOR_FR, _throttle_us);
    _writePWM(PIN_MOTOR_BL, _throttle_us);
    _writePWM(PIN_MOTOR_BR, _throttle_us);

    LOG_FMT("[MOTOR] Throttle: %i µs", _throttle_us);
}

void MotorMixer::stop()
{
    setThrottle(ESC_MIN_US);
    LOG("[MOTOR] STOP");
}

void MotorMixer::mix(uint16_t throttle, float roll, float pitch, float yaw)
{
    // X-Konfiguration:
    //      Vorne
    //  FL(CW)  FR(CCW)
    //  BL(CCW) BR(CW)
    //
    // Roll  positiv = rechts kippen → FL/BL mehr, FR/BR weniger
    // Pitch positiv = vorwaerts kippen → BL/BR mehr, FL/FR weniger

    float t = (float)throttle;
    
    _fl = (uint16_t)constrain(t - roll + pitch, ESC_MIN_US, ESC_MAX_US);
    _fr = (uint16_t)constrain(t + roll + pitch, ESC_MIN_US, ESC_MAX_US);
    _bl = (uint16_t)constrain(t - roll - pitch, ESC_MIN_US, ESC_MAX_US);
    _br = (uint16_t)constrain(t + roll - pitch, ESC_MIN_US, ESC_MAX_US);

    // Debug — was geht raus?
    LOG_FMT("[MIX] FL:%d FR:%d BL:%d BR:%d", _fl, _fr, _bl, _br);

    _writePWM(PIN_MOTOR_FL, _fl);
    _writePWM(PIN_MOTOR_FR, _fr);
    _writePWM(PIN_MOTOR_BL, _bl);
    _writePWM(PIN_MOTOR_BR, _br);

    LOG_FMT("[MOTOR] FL:%d FR:%d BL:%d BR:%d", _fl, _fr, _bl, _br);
}
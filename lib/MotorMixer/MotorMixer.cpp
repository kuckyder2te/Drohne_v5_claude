#include "MotorMixer.h"
#include "myLogger.h"
#include "config.h"
#include "hardware/pwm.h"

// ── Hilfsfunktion: Pin als PWM konfigurieren ───────────────
static void pwm_init_pin(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 20000);
    pwm_set_enabled(slice, true);
}

static uint16_t us_to_count(uint16_t us)
{
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

    stop();
    delay(2000);
    LOG("[MOTOR] ESC Initialisierung abgeschlossen");
}

void MotorMixer::setThrottle(uint16_t throttle_us)
{
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

// ── Einzelmotor Test ───────────────────────────────────────
void MotorMixer::setSingle(uint8_t motor, uint16_t throttle)
{
    uint16_t t = constrain(throttle, ESC_MIN_US, ESC_MAX_US);

    // Erst alle stoppen
    _writePWM(PIN_MOTOR_FL, ESC_MIN_US);
    _writePWM(PIN_MOTOR_FR, ESC_MIN_US);
    _writePWM(PIN_MOTOR_BL, ESC_MIN_US);
    _writePWM(PIN_MOTOR_BR, ESC_MIN_US);

    switch (motor)
    {
    case 1:
        _writePWM(PIN_MOTOR_FL, t);
        LOG_FMT("[MOTOR] FL: %d us (PIN %d)", t, PIN_MOTOR_FL);
        break;
    case 2:
        _writePWM(PIN_MOTOR_FR, t);
        LOG_FMT("[MOTOR] FR: %d us (PIN %d)", t, PIN_MOTOR_FR);
        break;
    case 3:
        _writePWM(PIN_MOTOR_BR, t);
        LOG_FMT("[MOTOR] BR: %d us (PIN %d)", t, PIN_MOTOR_BR);
        break;
    case 4:
        _writePWM(PIN_MOTOR_BL, t);
        LOG_FMT("[MOTOR] BL: %d us (PIN %d)", t, PIN_MOTOR_BL);
        break;
    default:
        LOG("[MOTOR] Unbekannter Motor!");
        break;
    }
}

void MotorMixer::mix(uint16_t throttle, float roll, float pitch, float yaw)
{
    float t = (float)throttle;

    _fl = (uint16_t)constrain(t - roll + pitch, ESC_MIN_US, ESC_MAX_US);
    _fr = (uint16_t)constrain(t + roll + pitch, ESC_MIN_US, ESC_MAX_US);
    _bl = (uint16_t)constrain(t - roll - pitch, ESC_MIN_US, ESC_MAX_US);
    _br = (uint16_t)constrain(t + roll - pitch, ESC_MIN_US, ESC_MAX_US);

    _writePWM(PIN_MOTOR_FL, _fl);
    _writePWM(PIN_MOTOR_FR, _fr);
    _writePWM(PIN_MOTOR_BL, _bl);
    _writePWM(PIN_MOTOR_BR, _br);
}

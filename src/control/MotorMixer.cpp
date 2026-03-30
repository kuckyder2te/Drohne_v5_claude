#include "control/MotorMixer.h"
#include "config.h"
#include "hardware/pwm.h"

// ── Hilfsfunktion: Pin als PWM konfigurieren ───────────────
static void pwm_init_pin(uint8_t pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    // 50 Hz: Systemtakt 125 MHz / (2500 * 1000) = 50 Hz
    pwm_set_clkdiv(slice, 125.0f);   // 125 MHz / 125 = 1 MHz
    pwm_set_wrap(slice, 20000);       // 1 MHz / 20000 = 50 Hz
    pwm_set_enabled(slice, true);
}

// ── µs in PWM-Counter-Wert umrechnen ──────────────────────
static uint16_t us_to_count(uint16_t us) {
    // Bei 1 MHz Takt = 1 Count pro µs
    return us;
}

void MotorMixer::_writePWM(uint8_t pin, uint16_t us) {
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, us_to_count(us));
}

void MotorMixer::begin() {
    pwm_init_pin(PIN_MOTOR_FL);
    pwm_init_pin(PIN_MOTOR_FR);
    pwm_init_pin(PIN_MOTOR_BL);
    pwm_init_pin(PIN_MOTOR_BR);

    // Alle ESCs mit Minimalthrottle initialisieren
    stop();
    delay(2000);  // ESCs kalibrieren lassen
    Serial.println("[MOTOR] ESC Initialisierung abgeschlossen");
}

void MotorMixer::setThrottle(uint16_t throttle_us) {
    // Sicherheitsgrenzen einhalten
    _throttle_us = constrain(throttle_us, ESC_MIN_US, ESC_MAX_US);

    _writePWM(PIN_MOTOR_FL, _throttle_us);
    _writePWM(PIN_MOTOR_FR, _throttle_us);
    _writePWM(PIN_MOTOR_BL, _throttle_us);
    _writePWM(PIN_MOTOR_BR, _throttle_us);

    Serial.print("[MOTOR] Throttle: ");
    Serial.print(_throttle_us);
    Serial.println(" µs");
}

void MotorMixer::stop() {
    setThrottle(ESC_MIN_US);
    Serial.println("[MOTOR] STOP");
}
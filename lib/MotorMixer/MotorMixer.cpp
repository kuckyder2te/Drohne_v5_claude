#include "MotorMixer.h"
#include "myLogger.h"
#include "config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ── Hilfsfunktion: Pin als PWM konfigurieren ───────────────
//
// Der Teiler wird aus der TATSAECHLICHEN Systemtaktfrequenz berechnet, nicht
// fest auf 125.0f gesetzt. Der Zaehler laeuft dadurch immer mit exakt 1 MHz,
// ein Zaehlschritt ist also 1 us und der Wrap bei 20000 ergibt 50 Hz - egal
// auf welchem Chip. Fest verdrahtete 125.0f galten nur fuer den RP2040 mit
// 125 MHz; auf dem RP2350 (Pico 2 / 2 W, 150 MHz) waeren daraus 60 Hz Rahmen
// und 0,833 us pro Zaehlschritt geworden, d.h. jeder ESC-Impuls rund 17 %
// zu kurz - ESC_MIN_US 1000 haette wie 833 us ausgesehen.
static void pwm_init_pin(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, (float)clock_get_hz(clk_sys) / 1000000.0f);
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

// ── ESC-Stromversorgung ────────────────────────────────────
// Low-Side-N-FET (IRLZ44N) an PIN_ESC_POWER: HIGH = ESCs am LiPo.
// Nur ein GPIO-Schreibzugriff, kein Log - damit von jedem Kern und aus
// jedem Zustand heraus aufrufbar.
void MotorMixer::powerOn()
{
    digitalWrite(PIN_ESC_POWER, HIGH);
    _powered = true;
}

void MotorMixer::powerOff()
{
    digitalWrite(PIN_ESC_POWER, LOW);
    _powered = false;
}

void MotorMixer::begin(bool autoPower)
{
    // Zuerst der Schalter, und zwar aus: der GPIO ist bis hierher hochohmig,
    // die ESCs duerfen erst Strom bekommen, wenn ein definiertes Signal
    // anliegt (siehe ESC_PWM_SETTLE_MS in config.h).
    pinMode(PIN_ESC_POWER, OUTPUT);
    powerOff();

    pwm_init_pin(PIN_MOTOR_FL);
    pwm_init_pin(PIN_MOTOR_FR);
    pwm_init_pin(PIN_MOTOR_BL);
    pwm_init_pin(PIN_MOTOR_BR);

    stop();

    if (!autoPower)
    {
        LOGGER_NOTICE("[MOTOR] PWM auf MIN - ESCs noch stromlos");
        return;
    }

    // Reihenfolge ist hier der ganze Zweck der Uebung: erst MIN ausgeben,
    // dann einschalten. Der ESC liest beim Hochlaufen sein Eingangssignal
    // und geht bei MIN in den Normalbetrieb; laege MAX an, ginge er in die
    // Kalibrierung.
    delay(ESC_PWM_SETTLE_MS);
    powerOn();
    delay(ESC_BOOT_MS);
    LOGGER_NOTICE("[MOTOR] ESC Initialisierung abgeschlossen (Strom EIN)");
}

void MotorMixer::setThrottle(uint16_t throttle_us)
{
    _throttle_us = constrain(throttle_us, ESC_MIN_US, ESC_MAX_US);
    _writePWM(PIN_MOTOR_FL, _throttle_us);
    _writePWM(PIN_MOTOR_FR, _throttle_us);
    _writePWM(PIN_MOTOR_BL, _throttle_us);
    _writePWM(PIN_MOTOR_BR, _throttle_us);
    LOGGER_NOTICE_FMT("[MOTOR] Throttle: %i µs", _throttle_us);
}

void MotorMixer::stop()
{
    setThrottle(ESC_MIN_US);
    LOGGER_NOTICE("[MOTOR] STOP");
}

// Not-Aus ohne Logausgabe und ohne Umweg ueber setThrottle(). Vier
// Registerschreibzugriffe, sonst nichts - damit ist das von jedem Kern
// und aus jedem Zustand heraus sicher aufrufbar und braucht keine
// Zusammenarbeit des anderen Kerns.
void MotorMixer::stopFast()
{
    _fl = _fr = _bl = _br = ESC_MIN_US;
    _throttle_us = ESC_MIN_US;
    _writePWM(PIN_MOTOR_FL, ESC_MIN_US);
    _writePWM(PIN_MOTOR_FR, ESC_MIN_US);
    _writePWM(PIN_MOTOR_BL, ESC_MIN_US);
    _writePWM(PIN_MOTOR_BR, ESC_MIN_US);
}

uint16_t MotorMixer::getMotorUs(uint8_t i) const
{
    switch (i) {
        case 0:  return _fl;
        case 1:  return _fr;
        case 2:  return _bl;
        default: return _br;
    }
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
        LOGGER_NOTICE_FMT("[MOTOR] FL: %d us (PIN %d)", t, PIN_MOTOR_FL);
        break;
    case 2:
        _writePWM(PIN_MOTOR_FR, t);
        LOGGER_NOTICE_FMT("[MOTOR] FR: %d us (PIN %d)", t, PIN_MOTOR_FR);
        break;
    case 3:
        _writePWM(PIN_MOTOR_BR, t);
        LOGGER_NOTICE_FMT("[MOTOR] BR: %d us (PIN %d)", t, PIN_MOTOR_BR);
        break;
    case 4:
        _writePWM(PIN_MOTOR_BL, t);
        LOGGER_NOTICE_FMT("[MOTOR] BL: %d us (PIN %d)", t, PIN_MOTOR_BL);
        break;
    default:
        LOGGER_NOTICE("[MOTOR] Unbekannter Motor!");
        break;
    }
}

void MotorMixer::mix(uint16_t throttle, float roll, float pitch, float yaw)
{
    float t = (float)throttle;
    float hi = (float)_maxUs;   // Pruefstand-Obergrenze, sonst ESC_MAX_US

    _fl = (uint16_t)constrain(t - roll + pitch, (float)ESC_MIN_US, hi);
    _fr = (uint16_t)constrain(t + roll + pitch, (float)ESC_MIN_US, hi);
    _bl = (uint16_t)constrain(t - roll - pitch, (float)ESC_MIN_US, hi);
    _br = (uint16_t)constrain(t + roll - pitch, (float)ESC_MIN_US, hi);

    _writePWM(PIN_MOTOR_FL, _fl);
    _writePWM(PIN_MOTOR_FR, _fr);
    _writePWM(PIN_MOTOR_BL, _bl);
    _writePWM(PIN_MOTOR_BR, _br);
}

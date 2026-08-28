#pragma once

// ── Debug Ausgabe ──────────────────────────────────────────
// Ziel der LOGGER_NOTICE()/LOGGER_NOTICE_FMT()-Ausgaben, beides gleichzeitig moeglich.
// Unabhaengig davon, wo die CLI-Shell haengt (siehe CLI_USE_BLUETOOTH).
//#define _SERIAL_LOG   // USB Serial - zuschalten, um Logs auch auf COM11 mitzulesen
#define _BT_LOG         // BT-UART (Serial1)

// ── Kanal der CLI-Shell ────────────────────────────────────
// Die Shell (SimpleSerialShell) ist ein Singleton mit genau EINEM Stream.
// Aktiv: CLI liegt auf Serial1 (BT-UART). Auskommentiert: CLI liegt auf
// Serial (USB) - dann funktioniert auch der serial-cli-test-Skill (COM11).
//#define CLI_USE_BLUETOOTH

// ── Betriebsmodus ────────────────────────────────────────────
// Die Firmware kennt nur noch den Normalbetrieb (Flugbetrieb, siehe
// src/mode/NormalMode.cpp). Alle Hardware-Testwerkzeuge sind eigenstaendige
// PlatformIO-Umgebungen unter src/tools/ (siehe src/tools/README, z.B.
// "pio run -e test_imu --target upload").

// ── Barometer im Flugbetrieb ───────────────────────────────
// AUSGESCHALTET. Grund: Der Lageregelkreis laeuft seit dem Dual-Core-Umbau auf
// Kern 1 und pollt den ICM-20948 mit ATTITUDE_RATE_HZ. MS5611 und IMU haengen
// am selben Wire-Bus; ein paralleler Barometer-Zugriff von Kern 0 wuerde den
// Bus teilen und braeuchte einen Mutex. Da der Baro ohnehin 90 s Warmlauf
// braucht und der Ultraschall im Tuning-Bereich (2-300 cm) genauer ist,
// bekommt Kern 1 den I2C-Bus stattdessen exklusiv - das haelt den
// Lage-Regelkreis frei von jedem Fremdzugriff.
//
// Wieder einschalten heisst: Mutex um alle Wire-Transaktionen in Barometer
// UND IMU legen (der Mutex muss requestFrom() samt aller folgenden read()
// umfassen, TwoWire::_buff ist gemeinsamer Zustand).
// Solange dies aus ist, ist der Ultraschall die einzige Hoehenquelle.
//#define BARO_ENABLED

// ── Dual-Core Lageregelung ─────────────────────────────────
// Kern 1: IMU lesen, Roll/Pitch-PID, Motor-Mixing.
// Kern 0: Ultraschall, Batterie, CLI, Hoehen-PID, Logging.
#define ATTITUDE_RATE_HZ     400
#define ATTITUDE_PERIOD_US   (1000000UL / ATTITUDE_RATE_HZ)

// Zeitkonstante des Komplementaerfilters. alpha wird daraus je Zyklus
// berechnet (alpha = tau/(tau+dt)) statt fest verdrahtet - bei 2,5 ms
// Zykluszeit waere das alte alpha=0.98 eine Zeitkonstante von 0,12 s.
#define IMU_TAU_S            0.5f

// Reisst der Herzschlag des jeweils anderen Kerns ab, gehen die Motoren aus.
// Kern 0 braucht mit Ultraschall (~50 ms) deutlich mehr Luft als Kern 1.
#define ATTITUDE_WATCHDOG_MS 150
#define CORE0_WATCHDOG_MS    500

// Kadenzen auf Kern 0 - vorher lief beides in jedem Durchlauf, was den
// Kern auf ~9 Hz gedrueckt hat.
#define ULTRA_UPDATE_MS      50
#define BARO_UPDATE_MS       200

// ── Pruefstand (1-Achsen-Wippe) ────────────────────────────
#define BENCH_THROTTLE_US    1300
#define BENCH_MAX_US         1600
#define BENCH_LIMIT_DEG      25.0f
#define BENCH_TIMEOUT_S      20

// Umrechnung Motordiagonale <-> Flugachse.
//
// Liegt die Wippenstange auf einer Motordiagonalen, treiben nur die beiden
// Motoren der anderen Diagonalen - dafuer mit dem Hebelarm a*sqrt(2) statt a.
// Das Traegheitsmoment ist beim symmetrischen X um beide Diagonalen genauso
// gross wie um Roll/Pitch (jeweils 4*m*a^2), es bleibt also nur der
// Unterschied in der Anregung:
//   Diagonale: 2 Motoren * c * a*sqrt(2) = 2*sqrt(2) * a*k*c
//   Roll:      4 Motoren * c * a         = 4         * a*k*c
// Verhaeltnis 2*sqrt(2)/4 = 1/sqrt(2) = 0,7071. Ein auf der Diagonalen
// ermittelter Beiwert wird mit diesem Faktor zu Roll/Pitch; umgekehrt faehrt
// der Diagonal-Pruefstand die Roll-Beiwerte durch diesen Faktor geteilt.
//
// Gilt fuer den symmetrischen X-Rahmen (gleich lange Arme, 90 Grad). Bei
// gestrecktem Rahmen (Deadcat) stimmt weder der 45-Grad-Anteil der Projektion
// noch die Traegheitsgleichheit - dann ist dieser Wert anzupassen.
#define DIAG_AXIS_GAIN       0.70710678f

// ── Relay-Feedback-Autotune (Aastroem-Haegglund) ───────────
#define TUNE_H_US            60.0f    // Relais-Amplitude
#define TUNE_EPS_DEG         1.0f     // Hysterese
#define TUNE_CYCLES          6        // auszuwertende Perioden
#define TUNE_WARMUP_CYCLES   2        // verworfene Einschwing-Perioden
#define TUNE_LIMIT_DEG       20.0f
#define TUNE_TIMEOUT_S       30
#define TUNE_NOSWITCH_MS     3000     // keine Umschaltung -> keine Schwingung

// ── Recorder ───────────────────────────────────────────────
// 3000 * 20 B = 60 kB in der .bss, bei REC_DECIMATION=2 sind das 15 s @200 Hz.
#define REC_CAPACITY         3000
#define REC_DECIMATION       2

// Zusaetzliche manuelle Temperaturkompensation der Druckmessung - deaktiviert.
// Die MS5611/5607-Formel kompensiert Temperatur bereits selbst ueber die
// C5/C6-PROM-Koeffizienten (siehe TEMP-Berechnung in Barometer::update()).
// Ein von 0 verschiedener Wert hier hat im Test (test_barometer, Sensor fest
// auf dem Tisch, >3 Minuten Laufzeit) eine kontinuierliche Hoehen-Drift von
// mehreren Metern erzeugt, obwohl Druck und Temperatur nahezu stabil waren -
// der Term war fuer dieses Board falsch bemessen/gepolt. Mit 0.0f blieb die
// Hoehe im selben Test trotz weiter leicht steigender Sensortemperatur stabil
// (nur noch normales ADC-Rauschen, kein Trend).
#define BARO_TEMP_COEFF 0.0f

// ESC PWM-Parameter (Standard: 1000–2000 µs)
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000
#define ESC_FREQ_HZ 50

// ── ESC-Stromversorgung (MOSFET an PIN_ESC_POWER) ──────────
// Die ESCs haengen nicht mehr direkt am LiPo, sondern hinter einem
// Logic-Level-N-FET an GP28. Damit bestimmt die Firmware, WAS der ESC beim
// Einschalten als Signal sieht - und genau das entscheidet seine Betriebsart:
//   MIN (1000 us) beim Einschalten -> normaler Betrieb
//   MAX (2000 us) beim Einschalten -> Kalibriermodus (siehe test_motors 'k')
// Vorher war beides gleichzeitig am Strom, der Kalibriermodus liess sich nur
// durch Ab- und Anstecken des LiPo erreichen.
#define ESC_PWM_SETTLE_MS 100   // PWM muss vor dem Einschalten stehen (>=2 Rahmen @50 Hz)
#define ESC_BOOT_MS 2000        // ESC-Eigeninitialisierung nach dem Einschalten

// ── Bluetooth HC-06 (UART0) ────────────────────────────────
#define BT_UART Serial
#define BT_BAUD 9600

// ── Flugparameter ──────────────────────────────────────────
#define TARGET_HEIGHT_CM 50
#define MAX_HEIGHT_CM 100
#define THROTTLE_STEP 50

// ── EEPROM-Adressen ────────────────────────────────────────
// Stehen jetzt ausschliesslich in include/storage/Settings.h - hier lagen
// sie frueher zusaetzlich, was beim Aendern still auseinanderlaufen konnte.

// ── PID Roll/Pitch ─────────────────────────────────────────
#define PID_KP_ROLL 0.5f // klein anfangen!
#define PID_KI_ROLL 0.0f
#define PID_KD_ROLL 0.0f
#define PID_KP_PITCH 0.5f
#define PID_KI_PITCH 0.0f
#define PID_KD_PITCH 0.0f

// ── Sollwerte Lage ─────────────────────────────────────────
#define TARGET_ROLL_DEG 0.0f
#define TARGET_PITCH_DEG 0.0f

// ── PID Startwerte ─────────────────────────────────────────
#define PID_KP_HEIGHT 2.0f
#define PID_KI_HEIGHT 0.0f
#define PID_KD_HEIGHT 0.0f

// ── Regelkreis ─────────────────────────────────────────────
#define PID_INTERVAL_MS 50      // 20 Hz Regelfrequenz
// Untergrenze Zielhoehe. Die US-Sensoren sitzen 48 mm ueber dem Boden, am
// Boden misst der Sensor also schon ~4,8 cm - eine Zielhoehe von 5 hiesse
// "auf dem Boden stehen bleiben", der Regler wuerde bei laufenden Motoren
// gegen den Boden regeln. 10 sind ~5 cm echte Bodenfreiheit.
#define THROTTLE_MIN_CM 10      // Untergrenze Zielhöhe
#define THROTTLE_OFFSET_US 1500 // Basis-Throttle: Motoren laufen an

// ── Anti-Windup: Liftoff-Schwelle ──────────────────────────
// Integral-Term aktiv erst wenn Ultraschall > dieser Wert.
// Anpassen falls Sensor nicht auf Bodenniveau montiert ist.
//
// Genau dieser Fall: die US-Sensoren sitzen 48 mm ueber dem Boden, der
// Ruhewert im Stand ist also ~4,8 cm. Bei den frueheren 6,0f lag das Gate
// nur 1,2 cm darueber - ein paar zu grosse Messwerte (HC-SR04-Rauschen,
// unebener Untergrund) haetten gereicht, um die Integratoren aufzuladen,
// waehrend die Drohne noch steht. 10,0f = ~5 cm echte Bodenfreiheit.
#define LIFTOFF_HEIGHT_CM 10.0f // cm über Boden = "abgehoben"
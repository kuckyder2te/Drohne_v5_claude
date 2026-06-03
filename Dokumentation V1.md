# Drohnen-Projekt Dokumentation
## Raspberry Pi Pico Quadrocopter

---

## 1. PROJEKTSTRUKTUR

```
drone_pico/
├── platformio.ini
├── include/
│   ├── config.h              ← Konfiguration & Test-Modi
│   ├── pins.h                ← Pin-Belegung
│   ├── myLogger.h            ← Log-Makros
│   ├── control/
│   │   ├── MotorMixer.h      ← Motor PWM Steuerung
│   │   └── PIDController.h   ← PID Regler
│   ├── sensor/
│   │   ├── Barometer.h       ← MS5607 Luftdruck
│   │   ├── IMU.h             ← MPU9250 Lagesensor
│   │   ├── Battery.h         ← Akkuüberwachung
│   │   └── Ultrasonic.h      ← HC-SR04 Höhenmessung
│   ├── comm/
│   │   ├── KeyboardInput.h   ← Tastatur/BT Eingabe
│   │   └── BluetoothConfig.h ← BT PID Konfiguration
│   └── storage/
│       └── Settings.h        ← EEPROM Speicherung
└── src/
    ├── main.cpp              ← Hauptprogramm
    └── (entsprechende .cpp Dateien)
```

---

## 2. HARDWARE

| Komponente | Modell | Protokoll | Pins |
|---|---|---|---|
| MCU | Raspberry Pi Pico (RP2040) | — | — |
| IMU | GY-91 (MPU9250) | I2C 0x68 | SDA=GP4, SCL=GP5 |
| Barometer | MS5607 (als GY-63/MS5611) | I2C 0x77 | SDA=GP4, SCL=GP5 |
| Bluetooth | HC-06 | UART0 | TX=GP0, RX=GP1 |
| Motor FL | Brushless + ESC | PWM | GP11 |
| Motor FR | Brushless + ESC | PWM | GP12 |
| Motor BL | Brushless + ESC | PWM | GP14 |
| Motor BR | Brushless + ESC | PWM | GP13 |
| Buzzer | — | GPIO | GP10 (100Ω) |
| Batterie ADC | — | ADC | GP26 |
| Ultraschall TRIG | HC-SR04 | GPIO | GP8 |
| Ultraschall ECHO1 | HC-SR04 #1 | GPIO | GP6 |
| Ultraschall ECHO2 | HC-SR04 #2 | GPIO | GP7 |

**Spannungsversorgung:**
- 3S LiPo → 2 Regler: 5V und 3.3V
- Spannungsteiler: R4=100k (Batterie→GP26), R5=20k (GP26→GND), Faktor=6.0
- Schottky D1 zwischen BEC 5V und Pico VBUS

---

## 3. PIN-BELEGUNG (pins.h)

```cpp
#define PIN_SDA 4           // I2C Daten
#define PIN_SCL 5           // I2C Takt
#define PIN_BT_TX 0         // Bluetooth TX
#define PIN_BT_RX 1         // Bluetooth RX
#define PIN_MOTOR_FL 11     // Motor vorne links
#define PIN_MOTOR_FR 12     // Motor vorne rechts
#define PIN_MOTOR_BL 14     // Motor hinten links
#define PIN_MOTOR_BR 13     // Motor hinten rechts
#define BUZZER 10           // Buzzer
#define BATTERY 26          // Batterie ADC
#define PIN_ULTRASONIC_TRIG1 8   // Ultraschall Trigger
#define PIN_ULTRASONIC_ECHO1 6   // Ultraschall Echo 1
#define PIN_ULTRASONIC_ECHO2 7   // Ultraschall Echo 2
```

---

## 4. KONFIGURATION (config.h)

### Test-Modi:
```cpp
// #define TEST_MOTORS          // Alle Motoren gleichzeitig
// #define TEST_MOTORS_SINGLE   // Einzelmotor Test
// #define TEST_BAROMETER       // Barometer Test
// #define TEST_KEYBOARD        // Tastatur Test
// #define TEST_I2C_SCAN        // I2C Scanner
// #define TEST_IMU             // IMU Test
// #define TEST_ULTRASONIC      // Ultraschall Test
// (alle auskommentiert = Normalbetrieb)
```

### PID Parameter:
```cpp
#define PID_KP_HEIGHT 2.0f    // Höhe Proportional
#define PID_KI_HEIGHT 0.03f   // Höhe Integral
#define PID_KD_HEIGHT 0.3f    // Höhe Differential
#define PID_KP_ROLL   0.5f    // Roll Proportional
#define PID_KP_PITCH  0.5f    // Pitch Proportional
```

### ESC Parameter:
```cpp
#define ESC_MIN_US 1000        // Minimum Throttle
#define ESC_MAX_US 2000        // Maximum Throttle
#define THROTTLE_OFFSET_US 1400 // Basis-Throttle
#define THROTTLE_STEP 50       // Schrittweite
```

### Batterie:
```cpp
#define BATTERY_WARN_V 10.5f   // Warnung
#define BATTERY_CRIT_V 10.0f   // Kritisch
```

### Barometer:
```cpp
#define BARO_TEMP_COEFF 0.5f   // Temperaturkompensation
```

---

## 5. MAIN.CPP AUFBAU

### Globale Objekte:
```cpp
MotorMixer motors;        // Motor PWM Steuerung
Barometer baro;           // Barometer MS5607
Battery battery;          // Akkuüberwachung
KeyboardInput keyboard;   // Tastatur/BT Eingabe
Ultrasonic ultrasonic;    // HC-SR04 Höhenmessung
PIDController pidHeight;  // PID Höhe (mit Offset)
PIDController pidRoll;    // PID Roll (ohne Offset)
PIDController pidPitch;   // PID Pitch (ohne Offset)
BluetoothConfig btConfig; // BT Konfiguration
Settings settings;        // EEPROM Einstellungen
IMU imu;                  // MPU9250 Lagesensor
```

### Zustandsvariablen:
```cpp
float targetHeightCm = 0.0f;  // Ziel-Höhe in cm
bool armed = false;            // ARM Status
uint32_t lastPidMs = 0;       // PID Timer
uint32_t lastPrintMs = 0;     // Status Timer
```

---

## 6. SETUP() ABLAUF

```
1. Serial.begin(115200)
2. battery.begin()        ← immer!
3. ultrasonic.begin()     ← immer!
4. btConfig.begin()       ← Bluetooth starten
5. pidHeight/Roll/Pitch.begin()
6. LOG "=== DROHNE PICO BOOT ===="
7. Test-Modus ODER Normalbetrieb:

NORMALBETRIEB:
   Wire.begin()           ← I2C starten
   baro.begin()           ← Barometer (30s Aufwärm + Stabilisierung)
   ultrasonic.begin()
   motors.begin()         ← ESC initialisieren (2s)
   keyboard.begin()
   settings.begin()       ← EEPROM laden
   imu.begin()            ← MPU9250
```

---

## 7. LOOP() ABLAUF

```
IMMER (alle Modi):
   battery.update()       ← Spannung messen, Buzzer

TEST_ULTRASONIC:
   ultrasonic.update()
   → Ausgabe alle 200ms

TEST_IMU:
   imu.update()
   → Ausgabe alle 100ms

TEST_MOTORS:
   BT/Serial lesen: + - s h
   → Alle Motoren gleichzeitig steuern

TEST_MOTORS_SINGLE:
   BT/Serial lesen: 1 2 3 4 + - s
   1=FL, 2=FR, 3=BR, 4=BL
   → Einzelmotor steuern

TEST_BAROMETER:
   baro.update()
   → Ausgabe alle 500ms

NORMALBETRIEB:
   baro.update()          ← Druck messen
   ultrasonic.update()    ← Höhe messen
   imu.update()           ← Lage messen
   btConfig.update()      ← BT Befehle
   Safety Check           ← IMU Fehler, Höhensprung
   keyboard.getKey()      ← Eingaben verarbeiten
   PID Regelkreis         ← alle 50ms
   Statusausgabe          ← alle 500ms
```

---

## 8. SENSOR-DETAILS

### Barometer (MS5607):
- Adresse: 0x77 (CSB→GND)
- Kompensationsformel: MS5607 (nicht MS5611!)
- Temperaturkompensation: `P_korr = P - (ΔT × 0.5)`
- Aufwärmzeit: 30s + thermische Stabilisierung
- Gibt: Druck (hPa), Temperatur (Chip-Eigentemperatur!), relative Höhe (cm)
- **Achtung:** Temperatur zeigt Chip-Temperatur ~40-48°C, nicht Umgebungstemperatur!

### Ultraschall (HC-SR04 × 2):
- TRIG: GP8 (gemeinsam für beide)
- ECHO1: GP6, ECHO2: GP7
- Sequenzielle Messung mit 30ms Pause
- Mittelwert wenn beide valid
- Fallback auf einzelnen Sensor
- Messbereich: 2-300 cm
- Genauigkeit: ±0.5 cm
- **Hauptsensor für Höhenregelung!**

### IMU (MPU9250):
- Adresse: 0x68 (SAO→GND)
- WHO_AM_I: 0x70, 0x71 oder 0x73 akzeptiert
- Gibt: Roll, Pitch (Grad), AccZ

### Batterie:
- ADC GP26, Spannungsteiler 100k/20k, Faktor 6.0
- Warnung <10.5V: 1x Beep alle 10s
- Kritisch <10.0V: 3x Beep alle 5s
- Non-blocking Buzzer!

---

## 9. PID REGELKREIS

```
Ziel: targetHeightCm (20cm nach ARM)
Ist:  ultrasonic.getAltitudeCm() (Fallback: baro)

pidHeight.compute(ziel, ist) → throttle (1000-2000µs)
pidRoll.compute(0°, imu.getRoll()) → rollCorr (±500)
pidPitch.compute(0°, imu.getPitch()) → pitchCorr (±500)

motors.mix(throttle, rollCorr, pitchCorr, 0):
   FL = throttle - roll + pitch
   FR = throttle + roll + pitch
   BL = throttle - roll - pitch
   BR = throttle + roll - pitch
```

---

## 10. BLUETOOTH BEFEHLE

```
a / A     → ARM (Motoren starten, Ziel: 20cm)
s / S     → DISARM (Motoren stoppen)
r / R     → Barometer rekalibrieren
h / H     → Hilfe anzeigen
↑         → Zielhöhe +10cm
↓         → Zielhöhe -10cm
?         → PID Werte anzeigen
P=x.x     → Höhe Kp setzen
I=x.x     → Höhe Ki setzen
D=x.x     → Höhe Kd setzen
RP=x.x    → Roll Kp setzen
RI=x.x    → Roll Ki setzen
RD=x.x    → Roll Kd setzen
PP=x.x    → Pitch Kp setzen
PI=x.x    → Pitch Ki setzen
PD=x.x    → Pitch Kd setzen
S=save    → EEPROM speichern
R=reset   → EEPROM zurücksetzen
```

---

## 11. SICHERHEITSFUNKTIONEN

```
Safety Check (nur wenn armed):
1. IMU Fehler → sofort DISARM
2. Höhensprung >500cm → sofort DISARM

Batterie:
3. <10.5V → Buzzer Warnung
4. <10.0V → Buzzer Kritisch (noch kein Auto-DISARM)

Barometer Fallback:
5. Ultraschall kein Signal → Barometer als Backup
```

---

## 12. BEKANNTE PROBLEME & LÖSUNGEN

| Problem | Ursache | Lösung |
|---|---|---|
| Barometer Drift | Temperaturkompensation nicht perfekt | Ultraschall als Hauptsensor |
| SCL Kurzschluss | Leiterbahn zu nah an GND | Korrigiert in neuer Platine |
| Spannungsteiler vertauscht | R4/R5 falsch gepolt | Korrigiert |
| MS5611 defekt | Überstrom durch SCL Kurzschluss | Neue Sensoren bestellt |
| WHO_AM_I 0x70 | MPU9250 Klon | 0x70 akzeptiert |
| Ultraschall 2 Sensoren | Gemeinsamer TRIG stört | 30ms Pause zwischen Messungen |

---

## 13. NOCH ZU TUN

- [ ] Motor FL Problem lösen (Einzelmotor Test)
- [ ] THROTTLE_OFFSET_US tunen für Abheben
- [ ] Erster Flugtest mit Propellern
- [ ] PID Tuning Roll/Pitch
- [ ] Zweiter Ultraschall TRIG Pin (GP9 reserviert)
- [ ] Kalman-Filter (Barometer + IMU) für höhere Flughöhen
- [ ] NRF24 Fernsteuerung (Phase 3)
- [ ] Yaw Regelung (AK8963 Magnetometer)

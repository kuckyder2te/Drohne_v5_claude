# Drohne Pico — Projektzusammenfassung
> Letzte Aktualisierung: 10.06.2026  
> Projektpfad: `H:\hobby\Projekte\Drohne\VS Code\Drohne_v5_Claude`

---

## Hardware

| Komponente      | Modell                  | Protokoll              |
|-----------------|-------------------------|------------------------|
| MCU             | Raspberry Pi Pico       | —                      |
| IMU             | MPU9250 (eigenständig)  | I2C (SPI in Phase 3)   |
| Barometer       | MS5611 (eigenständig)   | I2C (nur Luftdruck)    |
| Höhensensor     | HC-SR04 Ultraschall     | Trigger/Echo (Digital) |
| Funk            | NRF24L01                | SPI (Phase 2)          |
| Bluetooth       | HC-06                   | UART0 (ohne Pegelwandler getestet) |
| Motoren         | 4x Brushless + ESC      | PWM (nativer RP2040 SDK) |

### Pinbelegung (`include/pins.h`)

```cpp
PIN_MOTOR_FL  = 11    // Front Left  (CCW ↺)
PIN_MOTOR_FR  = 12    // Front Right (CW  ↻)
PIN_MOTOR_BL  = 14    // Back Left   (CW  ↻)
PIN_MOTOR_BR  = 13    // Back Right  (CCW ↺)

PIN_SDA       = 4     // I2C — MS5611 Barometer + MPU9250
PIN_SCL       = 5

PIN_TRIG1     = 8     // HC-SR04 Ultraschall Trigger
PIN_ECHO1     = 6     // HC-SR04 Ultraschall Echo

PIN_RADIO_CE  = 20    // NRF24L01
PIN_RADIO_CSN = 17

PIN_BT_TX     = 0     // HC-06 UART0
PIN_BT_RX     = 1

// SPI (MPU9250 + NRF24) — noch offen für Phase 3
PIN_SPI_MOSI  = ?
PIN_SPI_MISO  = ?
PIN_SPI_SCK   = ?
PIN_IMU_CS    = ?
```

### Motorkonfiguration (X-Frame)

```
      Vorne
  FL(CCW) FR(CW)
    ↺        ↻
      \    /
       \  /
       /  \
      /    \
    ↻        ↺
  BL(CW)  BR(CCW)
      Hinten
```

| Motor            | Pin    | Drehrichtung |
|------------------|--------|-------------|
| Front Left (FL)  | PIN 11 | CCW ↺ |
| Front Right (FR) | PIN 12 | CW  ↻ |
| Back Left (BL)   | PIN 14 | CW  ↻ |
| Back Right (BR)  | PIN 13 | CCW ↺ |

### Sensorhinweise
- MS5611: eigenständiges Modul, wird nur für **Luftdruck** verwendet
- MPU9250: eigenständiges Modul (kein CJMCU)
- HC-SR04: Höhenregelung per Ultraschall
- HC-06: VCC an **VBUS (5V)**, nicht 3.3V

---

## Projektstruktur

```
drone_pico/
├── platformio.ini
├── README.md
├── CHAT_SUMMARY.md         ← diese Datei
├── include/
│   ├── pins.h
│   ├── config.h
│   ├── comm/
│   │   ├── BluetoothConfig.h
│   │   └── KeyboardInput.h
│   ├── control/
│   │   ├── MotorMixer.h
│   │   └── PIDController.h
│   ├── sensor/
│   │   ├── Barometer.h
│   │   └── IMU.h
│   └── storage/
│       └── Settings.h
└── src/
    ├── main.cpp
    ├── comm/
    │   ├── BluetoothConfig.cpp
    │   └── KeyboardInput.cpp
    ├── control/
    │   ├── MotorMixer.cpp
    │   └── PIDController.cpp
    ├── sensor/
    │   ├── Barometer.cpp
    │   └── IMU.cpp
    └── storage/
        └── Settings.cpp
```

---

## platformio.ini (aktuell)

```ini
[env:pico]
platform = raspberrypi
board = pico
framework = arduino

monitor_speed = 115200
upload_protocol = picotool

lib_deps =
    bakercp/Logger@^1.0.3
    nrf24/RF24@^1.4.8
    robtillaart/MS5611@^0.3.9
    adafruit/Adafruit Unified Sensor@^1.1.6
    hideakitai/MPU9250@^0.4.8
    adafruit/Adafruit BusIO@^1.14.1

build_flags =
    -DGLOBAL_DEBUG
;   -DLOG_TIMESTAMP
;   -D_DEBUG_=VERBOSE
    -D_DEBUG_=NOTICE
;   -D_DEBUG_=WARNING
;   -D_DEBUG_=FATAL
```

> **Entfernte Libs:** FastPID (eigene PID-Impl.), TaskManager (millis() reicht), RP2040_PWM (nativer SDK)

---

## Entwicklungsstand

### ✅ Abgeschlossen

| Schritt | Inhalt |
|---------|--------|
| 1 | Projektstruktur angelegt |
| 2 | ESC/Motor-Test (`TEST_MOTORS`) |
| 3 | Barometer MS5611 (`TEST_BAROMETER`) |
| 4 | Keyboard-Input (`TEST_KEYBOARD`) |
| 5 | PIDController implementiert (Roll, Pitch, Yaw, Höhe) |
| 6 | BluetoothConfig — PID per BT einstellbar |
| 7 | Settings — EEPROM-Speicherung der PID-Werte |
| 8a | IMU MPU9250 — I2C stabil mit Bus-Recovery |
| 8b | MotorMixer — mix() implementiert, Regelkreis in loop() |

### 🔜 Offen

| Schritt | Inhalt |
|---------|--------|
| 8c | Pico 2W BT vorbereiten (Board vorhanden!) |
| 8d | Platinen-Design in KiCad |
| 8e | Schwebeflug-Test |
| Phase 2 | NRF24 Radio — Fernsteuerung |
| Phase 3 | MPU9250 auf SPI umstellen |

---

## Bluetooth-Befehle (HC-06)

| Befehl | Funktion |
|--------|----------|
| `P=1.5` | Kp setzen |
| `I=0.05` | Ki setzen |
| `D=0.2` | Kd setzen |
| `?` | Aktuelle PID-Werte abfragen |
| `a` | ARM |
| `s` | DISARM (sofort!) |
| `r` | Rekalibrierung |

---

## Bekannte Probleme & Lösungen

### I2C Bus-Freeze (IMU)
**Problem:** Nach ~5 Minuten I2C Fehler Code 5 — Bus hängt.  
**Lösung:** Manuelle Bus-Recovery mit 9 Clock-Pulsen in `IMU::update()`:
```cpp
Wire.end();
// 9 Clock-Pulse manuell
pinMode(PIN_SDA, OUTPUT); pinMode(PIN_SCL, OUTPUT);
digitalWrite(PIN_SDA, HIGH);
for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
}
// STOP Condition + Wire neu starten
```
**Ergebnis:** 34+ Minuten stabil getestet ✅

### Wire.begin() Konflikt
**Problem:** IMU und Barometer rufen beide `Wire.begin()` auf.  
**Lösung:** In `IMU::begin()` kein `Wire.begin()` — Barometer initialisiert den Bus zuerst. Im `TEST_IMU` Modus explizit in `setup()` aufrufen.

### TaskManager entfernt
`millis()`-basierte Zeitsteuerung ist ausreichend und stabiler.

---

## Entwicklungsregeln (aus AI.md)

- Keine neuen Libraries ohne Rücksprache
- Keine Änderungen an `platformio.ini` ohne Rücksprache
- Speicher- und Performance-optimierter Code
- Immer nur **einen Test-Modus** gleichzeitig aktiv
- **Finger auf `s`** beim Testen — sofort DISARM!
- Propeller beim Code-Test immer abnehmen!

---

## Nächster Schritt

Schwebeflug-Vorbereitung:
1. Normalbetrieb aktivieren (alle `TEST_*` auskommentiert)
2. Flash + Boot → 90s Aufwärmzeit
3. `r` → Rekalibrierung
4. `a` → ARM (ohne Propeller zuerst!)
5. Board kippen → Motorwerte prüfen
6. Wenn OK → Propeller montieren → erster Flugtest

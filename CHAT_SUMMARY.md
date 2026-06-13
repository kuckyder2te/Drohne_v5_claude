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
| Phase 3 | Yaw-Stabilisierung — Magnetometer + Gyro-Integration |

---

## Bluetooth-Befehle (HC-06)

### Steuerung
| Befehl | Funktion |
|--------|----------|
| `a` | ARM (2× drücken innerhalb 3s) |
| `s` | DISARM (sofort!) |
| `r` | Barometer rekalibrieren |
| `l` | Statusausgabe ein/aus |
| `h` | Hilfe anzeigen |
| `+` | Zielhöhe +10 cm |
| `-` | Zielhöhe -10 cm |

### PID-Tuning (Höhe)
| Befehl | Funktion |
|--------|----------|
| `P=1.5` | Höhe Kp setzen |
| `I=0.05` | Höhe Ki setzen |
| `D=0.2` | Höhe Kd setzen |

### PID-Tuning (Roll/Pitch)
| Befehl | Funktion |
|--------|----------|
| `RP=0.5` | Roll Kp setzen |
| `RI=0.0` | Roll Ki setzen |
| `RD=0.0` | Roll Kd setzen |
| `PP=0.5` | Pitch Kp setzen |
| `PI=0.0` | Pitch Ki setzen |
| `PD=0.0` | Pitch Kd setzen |

### Speichern & Abfragen
| Befehl | Funktion |
|--------|----------|
| `?` | Aktuelle PID-Werte abfragen |

> **Hinweis:** `S` (Save) und `R` (Reset PID) sind aktuell nicht per BT nutzbar —
> Einzelbuchstabe `s` = DISARM, `r` = Rekalibrierung. Fix steht aus.

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

### MPU9250 antwortet nicht — WHO_AM_I: 0xFF

**Symptom:** `[IMU] WHO_AM_I: 0xFF` → `FEHLER: IMU!` — I2C-Bus selbst funktioniert (Barometer 0x77 antwortet).  
**Bedeutung:** 0xFF = kein ACK vom Chip — Gerät physisch nicht erreichbar.

**Diagnoseergebnis (2026-06-11):**  
I2C-Scan zeigt: `0x76` + `0x77` (beide Barometer-Bereich) — kein 0x68/0x69.  
→ MPU9250 ist **nicht auf dem Bus**. Chip defekt oder nicht verbunden.  
Hinweis: 0x76 = MS5611 auf Alternativadresse (CSB=GND) oder zweiter Drucksensor (z.B. BMP280).

**Schnelldiagnose:**
1. Multimeter: 3,3 V zwischen VCC und GND des IMU-Boards?
2. Durchgangsprüfung: SDA→GPIO4, SCL→GPIO5 (Widerstand < 5 Ω?)
3. I2C-Scan (`TEST_I2C_SCAN`): Erscheint 0x68 oder 0x69?
4. Anderes MPU9250-Board anschließen → reagiert es?

**Häufige Ursachen (Reihenfolge nach Wahrscheinlichkeit):**

| Ursache | Hinweis |
|---------|---------|
| ESD-Schaden | MPU9250 extrem empfindlich — kurzes Anfassen ohne Erdung reicht |
| Kalte Lötstelle | Tritt oft nach Erschütterung auf, besonders Cheap-Boards |
| VCC-Pin lose | Breadboard-Kontakt geprüft? |
| Chip-Defekt | Günstige GY-91/GY-6500-Boards mit Fake-Chips fallen plötzlich aus |

**Softwarereaktion:** ARM wird verweigert wenn `!imu.isReady()` — keine Motorreaktion.  
**Adressvarianten:** MPU9250 → 0x68 (AD0=GND) oder 0x69 (AD0=VCC). WHO_AM_I akzeptiert: 0x71, 0x73, 0x70.  
**Anderen Chip-Typ?** ICM-20689 (0x98), MPU-6050 (0x68, WHO_AM_I=0x68) → Prüfung in `IMU.cpp:57` erweitern.

---

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

### ESD-Schutz & Komponentenhandhabung

**Besonders gefährdete Bauteile:**

| Komponente | Empfindlichkeit |
|---|---|
| RP2040 (Pico) | Sehr hoch — CMOS-Prozessor |
| MPU9250 | Hoch — MEMS-Struktur |
| MS5611 | Hoch — Drucksensor-Membran |
| MOSFETs in ESCs | Mittel-hoch |

**Maßnahmen (Priorität):**
1. **Antistatik-Armband** beim Arbeiten tragen (bestellt ✅)
2. Antistatik-Arbeitsmatte (~10–15 €) empfohlen
3. Spare-Platinen in **antistatischen Tüten** lagern
4. Vor dem Anfassen: kurz geerdetes Metallteil berühren (Heizung, PC-Gehäuse)
5. **USB-Kabel ziehen** bevor am Board gearbeitet wird
6. Lötreihenfolge: zuerst GND, zuletzt VCC

**Erfahrung aus diesem Projekt:**  
Drei MPU9250-Boards durch ESD zerstört — der 4. Chip (2026-06-12) war OK nachdem beide Sensoren
(0x68 + 0x77) im I2C-Scan wieder erschienen.  
→ GY-91 (MPU9250 + BMP280) als Alternativboard geprüft, aber MS5611 bleibt bevorzugt (höhere Barometergenauigkeit).

---

### Ein-/Ausschalten — Reihenfolge & Hinweise

**Einschalten:**
1. Drohne **ruhig hinlegen** — IMU kalibriert 100 Gyro-Samples beim Start
2. **Akku anschließen** → ESCs piepen (Initialisierung abwarten)
3. **~30 s warten** — Barometer Warmup; erst dann `a` (ARM)
4. USB optional danach für Serial-Monitor

**Ausschalten:**
1. **`s` drücken** (DISARM) — Motoren auf Minimum
2. Dann erst Akku trennen
3. Nie unter Last trennen — Lichtbogen schadet Kontakten und ESCs

**ESD beim Stecken:**
- Stecker immer am Gehäuse anfassen, nie an Pins
- XT60-Stecker zügig verbinden (langsames Stecken → Lichtbogen)
- **Anti-Spark-Widerstand** am XT60 empfehlenswert (~2 €)
- I2C-Kabel (MPU9250, MS5611) **niemals** im laufenden Betrieb stecken → Chip-Schaden

**LiPo 3S Spannungsgrenzen:**

| Spannung | Zustand |
|---|---|
| 12,6 V | Voll geladen |
| 11,1 V | Nennspannung |
| 10,5 V | Warnung (1 Beep) — landen! |
| 10,0 V | Kritisch (3 Beeps) — sofort aus! |
| < 9,9 V | Zellschaden dauerhaft |

> Warnschwellen implementiert in `src/sensor/Battery.cpp`

---

## Nächster Schritt

Schwebeflug-Vorbereitung:
1. Normalbetrieb aktivieren (alle `TEST_*` auskommentiert)
2. Flash + Boot → 90s Aufwärmzeit
3. `r` → Rekalibrierung
4. `a` → ARM (ohne Propeller zuerst!)
5. Board kippen → Motorwerte prüfen
6. Wenn OK → Propeller montieren → erster Flugtest


---

## Flugversuch-Parameter

### Versuch 1 (2026-06-10)
**PID-Werte:**
| Achse  | Kp     | Ki     | Kd     |
|--------|--------|--------|--------|
| Höhe   | 2.0000 | 0.0500 | 0.5000 |
| Roll   | 0.5000 | —      | —      |
| Pitch  | 0.5000 | —      | —      |

**Ergebnis:** ...
**Änderungen für V2:** ...


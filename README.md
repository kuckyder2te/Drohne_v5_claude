# 🚁 Drohnenprojekt — Raspberry Pi Pico Quadrocopter

**Entwickler:** Willy  
**Stand:** Schritt 8 von 8 — Schwebeflug-Test  
**Ziel Phase 1:** Stabiler Schwebeflug bis 100 cm Höhe

---

## Inhaltsverzeichnis

1. [Hardware](#hardware)
2. [Pinbelegung](#pinbelegung)
3. [Projektstruktur](#projektstruktur)
4. [Bibliotheken](#bibliotheken)
5. [Konfiguration](#konfiguration)
6. [Test-Modi](#test-modi)
7. [Bluetooth-Befehle](#bluetooth-befehle)
8. [Entwicklungsstand](#entwicklungsstand)
9. [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)
10. [Entwicklungsregeln](#entwicklungsregeln)

---

## Hardware

| Komponente | Modell | Protokoll |
|---|---|---|
| MCU | Raspberry Pi Pico | — |
| IMU + Barometer | CJMCU-10DOF | I2C / SPI |
| IMU | MPU9250 (auf CJMCU-10DOF) | SPI (Phase 3) |
| Barometer | MS5611 (auf CJMCU-10DOF) | I2C |
| Funk | NRF24L01 | SPI (Phase 2) |
| Bluetooth | HC-06 | UART0 |
| Motoren | 4x Brushless + ESC | PWM (nativer RP2040 SDK) |

### CJMCU-10DOF Anschluss

Das CJMCU-10DOF ist ein Kombi-Board mit MPU9250 und MS5611 auf einer Platine.

| CJMCU-10DOF Pin | Pico Pin | Funktion |
|---|---|---|
| VCC | 3.3V | Stromversorgung |
| GND | GND | Masse |
| SDA | PIN 4 | I2C — MS5611 |
| SCL | PIN 5 | I2C — MS5611 |
| PS | 3.3V | Protocol Select → I2C-Modus |
| NCS | 3.3V | Chip Select → I2C-Modus |
| SDO/MISO | PIN ? | SPI — MPU9250 (Phase 3) |
| SDI/MOSI | PIN ? | SPI — MPU9250 (Phase 3) |
| SCK | PIN ? | SPI — MPU9250 (Phase 3) |
| CS | PIN ? | Chip Select MPU9250 (Phase 3) |

> ⚠️ **Wichtig:** PS und NCS müssen an 3.3V angeschlossen sein — sonst wird der MS5611 nicht erkannt!

### HC-06 Bluetooth Anschluss

| HC-06 Pin | Pico Pin | Hinweis |
|---|---|---|
| VCC | VBUS (5V) | Nicht 3.3V! |
| GND | GND | — |
| TX | PIN 1 (RX) | In der Praxis ohne Pegelwandler getestet |
| RX | PIN 0 (TX) | — |

### Drehrichtungen (X-Konfiguration)

```
      Vorne
  FL(CW)  FR(CCW)
    ↻        ↺
      \    /
       \  /
       /  \
      /    \
    ↺        ↻
  BL(CCW) BR(CW)
      Hinten
```

| Motor | Pin | Drehrichtung |
|---|---|---|
| Front Left (FL) | PIN 11 | CW ↻ |
| Front Right (FR) | PIN 12 | CCW ↺ |
| Back Left (BL) | PIN 14 | CCW ↺ |
| Back Right (BR) | PIN 13 | CW ↻ |

---

## Pinbelegung

Alle Pins zentral in `include/pins.h` — **einzige Wahrheit für alle Pin-Definitionen!**

```cpp
// Motoren (PWM — nativer RP2040 SDK)
PIN_MOTOR_FL  = 11    // Front Left
PIN_MOTOR_FR  = 12    // Front Right
PIN_MOTOR_BL  = 14    // Back Left
PIN_MOTOR_BR  = 13    // Back Right

// I2C (MS5611 Barometer)
PIN_SDA       = 4
PIN_SCL       = 5

// SPI (MPU9250 + NRF24) — Phase 3
PIN_SPI_MOSI  = ?     // noch offen
PIN_SPI_MISO  = ?     // noch offen
PIN_SPI_SCK   = ?     // noch offen
PIN_IMU_CS    = ?     // noch offen

// Radio NRF24 — Phase 2
PIN_RADIO_CE  = 20
PIN_RADIO_CSN = 17

// Bluetooth HC-06 (UART0)
PIN_BT_TX     = 0
PIN_BT_RX     = 1
```

---

## Projektstruktur

```
drone_pico/
├── platformio.ini          ← PlatformIO Konfiguration
├── README.md               ← Diese Datei
├── include/                ← Alle Header-Dateien (.h)
│   ├── pins.h              ← Hardware-Pinbelegung (einzige Wahrheit!)
│   ├── config.h            ← Parameter, Konstanten, Test-Modi
│   ├── myLogger.h          ← Eigener Logger (basiert auf bakercp/Logger)
│   ├── comm/
│   │   ├── BluetoothConfig.h
│   │   └── KeyboardInput.h
│   ├── control/
│   │   ├── MotorMixer.h
│   │   └── PIDController.h
│   ├── sensor/
│   │   ├── Barometer.h
│   │   └── IMU.h           ← Platzhalter für Phase 3
│   └── storage/
│       └── Settings.h
└── src/                    ← Alle Implementierungen (.cpp)
    ├── main.cpp
    ├── myLogger.cpp
    ├── comm/
    │   ├── BluetoothConfig.cpp
    │   └── KeyboardInput.cpp
    ├── control/
    │   ├── MotorMixer.cpp
    │   └── PIDController.cpp
    ├── sensor/
    │   ├── Barometer.cpp
    │   └── IMU.cpp         ← Platzhalter für Phase 3
    └── storage/
        └── Settings.cpp
```

> **Konvention:** `.h` in `include/`, `.cpp` in `src/`. PlatformIO findet Header automatisch.

---

## Bibliotheken

Alle Bibliotheken in `platformio.ini`. **Keine neuen Bibliotheken ohne Rücksprache!**

| Bibliothek | Version | Verwendung |
|---|---|---|
| bakercp/Logger | ^1.0.3 | Basis für myLogger |
| nrf24/RF24 | ^1.4.8 | NRF24L01 Funk (Phase 2) |
| robtillaart/MS5611 | ^0.3.9 | Barometer |
| adafruit/Adafruit Unified Sensor | ^1.1.6 | Sensor-Abstraktion |
| hideakitai/MPU9250 | ^0.4.8 | IMU (Phase 3) |
| adafruit/Adafruit BusIO | ^1.14.1 | I2C/SPI Abstraktion |

**Entfernte Bibliotheken:**

| Bibliothek | Grund |
|---|---|
| ~~mike-matera/FastPID~~ | Eigene PID-Implementierung — keine Koeffizient-Einschränkungen |
| ~~khoih-prog/RP2040_PWM~~ | Nativer RP2040-SDK PWM stabiler und einfacher |
| ~~hideakitai/TaskManager~~ | Zeitsteuerung per `millis()` ausreichend |

---

## Konfiguration

### `platformio.ini`

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

### ESC PWM Parameter

| Parameter | Wert |
|---|---|
| Frequenz | 50 Hz |
| Minimum (Stop) | 1000 µs |
| Maximum (Vollgas) | 2000 µs |
| Basis-Offset (Anlauf) | 1100 µs |
| Throttle-Schritt | 50 µs |

### Barometer Konfiguration

| Parameter | Wert |
|---|---|
| Oversampling | OSR_ULTRA_HIGH |
| Filter | Gleitender Mittelwert, 20 Samples |
| Aufwärmzeit | 90 Sekunden |
| Kalibrierung | 20 Messungen à 100ms |
| Genauigkeit (Stillstand) | ± 1 cm |
| I2C Adresse MS5611 | 0x77 |
| I2C Adresse MPU9250 | 0x68 |

> ℹ️ Der MS5611 ist sehr empfindlich — Handbewegungen in der Nähe beeinflussen die Messung. Im Freien deutlich stabiler als im Innenraum.

### PID-Regler

Eigene Implementierung ohne externe Bibliothek.

| Parameter | Standardwert | Einstellbar per |
|---|---|---|
| Kp | 2.0 | Bluetooth |
| Ki | 0.03 | Bluetooth |
| Kd | 0.3 | Bluetooth |
| Regelfrequenz | 20 Hz (50ms) | config.h |
| Anti-Windup | ±500 | PIDController.h |
| Basis-Throttle | 1100 µs | config.h |

PID-Werte werden im Flash gespeichert und beim Start automatisch geladen.

### EEPROM-Layout

| Adresse | Inhalt | Größe |
|---|---|---|
| 0x00 | Kp | 4 Byte (float) |
| 0x04 | Ki | 4 Byte (float) |
| 0x08 | Kd | 4 Byte (float) |
| 0x0C | Validierungs-Marker (0xAB) | 1 Byte |

---

## Test-Modi

Test-Modi in `include/config.h` per `#define` aktivieren.
**Immer nur einen Test-Modus gleichzeitig!**

```cpp
// #define TEST_MOTORS       // Schritt 2: ESC/Motor Test
// #define TEST_BAROMETER    // Schritt 3: MS5611 Test
// #define TEST_KEYBOARD     // Schritt 4: Tastatur Test
// #define TEST_I2C_SCAN     // Diagnose: I2C Bus Scanner
// Alle auskommentiert = NORMALBETRIEB
```

### TEST_MOTORS
Motortest per Serial-Befehl. **Propeller abnehmen!**
- `+` → Throttle +50 µs
- `-` → Throttle -50 µs
- `s` → STOP

### TEST_BAROMETER
Barometer-Ausgabe alle 500ms: Höhe, Druck, Temperatur.

### TEST_KEYBOARD
Pfeiltasten-Erkennung + Barometer-Ausgabe.
- Pfeil hoch → Zielhöhe +10 cm
- Pfeil runter → Zielhöhe -10 cm
- `a` → ARM
- `r` → Barometer rekalibrieren
- `s` → DISARM
- `h` → Hilfe

> ⚠️ **Pfeiltasten:** Im PlatformIO Terminal (`pio device monitor`) — nicht im eingebauten Serial Monitor!

### TEST_I2C_SCAN
Scannt den I2C-Bus:
- `0x68` → MPU9250 ✅
- `0x77` → MS5611 ✅

### NORMALBETRIEB (alle auskommentiert)
Vollständiger Regelkreis mit PID-Höhenregelung.

**Befehle:**
- Pfeil hoch → Zielhöhe +10 cm
- Pfeil runter → Zielhöhe -10 cm
- `a` → ARM (Rekalibrierung + Motoren ein, Ziel 20 cm)
- `s` → DISARM (Motoren sofort stopp)
- `r` → Barometer rekalibrieren + PID reset
- `h` → Hilfe

---

## Bluetooth-Befehle

Verbindung per PuTTY. **Baudrate: 9600**

| Befehl | Funktion | Beispiel |
|---|---|---|
| `?` | Aktuelle PID-Werte anzeigen | `?` |
| `P=x.x` | Kp setzen | `P=2.0` |
| `I=x.x` | Ki setzen | `I=0.03` |
| `D=x.x` | Kd setzen | `D=0.3` |
| `S` | Werte im Flash speichern | `S` |
| `R` | EEPROM zurücksetzen | `R` |

**PuTTY Einstellungen:**
- Connection type: Serial
- Speed: 9600
- Terminal → Local echo: Force on
- Terminal → Local line editing: Force on

---

## Entwicklungsstand

| Schritt | Inhalt | Status |
|---|---|---|
| 1 | Projektstruktur | ✅ Fertig |
| 2 | PWM & Motortest | ✅ Fertig |
| 3 | Barometer MS5611 | ✅ Fertig |
| 4 | Tastatur-Eingabe | ✅ Fertig |
| 5 | PID-Regler (eigene Impl.) | ✅ Fertig |
| 6 | Bluetooth PID-Config | ✅ Fertig |
| 7 | Flash-Speicher (EEPROM) | ✅ Fertig |
| 8 | Schwebeflug-Test | 🔄 Aktuell |

**Geplante Folgephasen:**

| Phase | Inhalt |
|---|---|
| 2 | Manuelle Steuerung per NRF24 Fernbedienung |
| 3 | Lage-Stabilisierung per IMU (MPU9250) |
| 4 | GPS-gestütztes Positions-Halten |
| 5 | Autonome Flugrouten |

---

## Bekannte Probleme & Lösungen

### PWM — RP2040_PWM Bibliothek
**Problem:** Kompiliert nicht zuverlässig unter PlatformIO.
**Lösung:** Nativer RP2040-SDK PWM direkt verwendet. Bibliothek entfernt.

### MS5611 nicht gefunden
**Problem:** `[BARO] ERROR: MS5611 nicht gefunden!`
**Lösungen:**
1. PS-Pin und NCS-Pin → an 3.3V anschließen
2. SDA/SCL vertauscht → Verkabelung prüfen
3. I2C-Scanner starten: `TEST_I2C_SCAN`

### Barometer-Drift im Innenraum
**Problem:** Höhenwerte driften nach dem Start stark (bis ±100 cm).
**Ursache:** MS5611 extrem empfindlich — Selbsterwärmung, Luftzug, Handbewegungen in der Nähe.
**Lösung:**
- 90s Aufwärmzeit vor Kalibrierung
- Rekalibrierung (`r`) direkt vor dem Armen
- Für präzise Tests: im Freien testen

### FastPID — Ungültige Koeffizienten
**Problem:** FastPID meldet Fehler bei normalen PID-Werten wegen interner Koeffizient-Grenzen.
**Lösung:** Eigene PID-Implementierung. FastPID entfernt.

### Pfeiltasten werden nicht erkannt
**Problem:** Eingebauter Serial Monitor überträgt keine ANSI Escape-Sequenzen.
**Lösung:** `pio device monitor` im VSCode Terminal verwenden.

### Pico wird von Windows nicht erkannt
**Problem:** Kein USB-Piepsen, kein COM-Port nach dem Flashen.
**Lösung:** `flash_nuke.uf2` auf den Pico laden (BOOTSEL-Modus), dann neu flashen.
Download: https://datasheets.raspberrypi.com/soft/flash_nuke.uf2

### HC-06 — PuTTY Verbindung schlägt fehl
**Problem:** "Zeitlimit für die Semaphore wurde erreicht"
**Lösung:** Erst HC-06 in Windows Bluetooth aktiv verbinden (LED leuchtet dauerhaft), dann PuTTY öffnen.

---

## Entwicklungsregeln

- Schritt für Schritt — kein nächster Schritt bevor der aktuelle funktioniert
- Keine neuen Bibliotheken ohne Rücksprache
- Fehler zuerst beheben, dann weitermachen
- `pins.h` ist die einzige Wahrheit für alle Pin-Definitionen
- `.h` in `include/`, `.cpp` in `src/`
- Test-Modi nie im Produktivbetrieb aktiv lassen
- PID-Werte nach Änderung immer mit `S` speichern

---

## Test- und Prüfergebnisse


## Test- und Prüfergebnisse

| Arm Color | Motor nut | Position | PIN | Direction |
|---|---|---|---|---|
| red |black | FL | 11 | CW |
| red | red | FR | 12 | CCW |
|white | red | BL | 14 | CCW |
|white | black | BR | 13 | CW |

HC6 Name HC-06 PIN 1234


## Änderungen für den Einsatz den PICO 2W

3. Windows Long Path aktivieren — wichtig für den Earle Philhower Core!
Im Terminal als Administrator:
reg add HKLM\SYSTEM\CurrentControlSet\Control\FileSystem /v LongPathsEnabled /t REG_DWORD /d 1 /f

# 🚁 Drohnenprojekt — Raspberry Pi Pico Quadrocopter

**Entwickler:** Willy  
**Stand:** Schritt 4 von 8  
**Ziel Phase 1:** Stabiler Schwebeflug bis 100 cm Höhe

---

## Inhaltsverzeichnis

1. [Hardware](#hardware)
2. [Pinbelegung](#pinbelegung)
3. [Projektstruktur](#projektstruktur)
4. [Bibliotheken](#bibliotheken)
5. [Test-Modi](#test-modi)
6. [Entwicklungsstand](#entwicklungsstand)
7. [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)
8. [Nächste Schritte](#nächste-schritte)

---

## Hardware

| Komponente | Modell | Protokoll |
|---|---|---|
| MCU | Raspberry Pi Pico | — |
| IMU + Barometer | CJMCU-10DOF | I2C / SPI |
| IMU | MPU9250 (auf CJMCU-10DOF) | SPI |
| Barometer | MS5611 (auf CJMCU-10DOF) | I2C |
| Funk | NRF24L01 | SPI |
| Bluetooth | HC-06 | UART |
| Motoren | 4x Brushless + ESC | PWM |

### CJMCU-10DOF Anschluss

Das CJMCU-10DOF ist ein Kombi-Board mit MPU9250 und MS5611 auf einer Platine.

| CJMCU-10DOF Pin | Funktion | Hinweis |
|---|---|---|
| VCC | 3.3V | Pico liefert 3.3V |
| GND | GND | — |
| SDA | I2C — MS5611 | PIN 4 |
| SCL | I2C — MS5611 | PIN 5 |
| PS | Protocol Select | → 3.3V für I2C-Modus |
| NCS | Chip Select MS5611 | → 3.3V für I2C-Modus |
| SDO/MISO | SPI — MPU9250 | Phase 3 |
| SDI/MOSI | SPI — MPU9250 | Phase 3 |
| SCK | SPI — MPU9250 | Phase 3 |
| CS | Chip Select MPU9250 | Phase 3 |

> ⚠️ **Wichtig:** PS und NCS müssen an 3.3V angeschlossen sein, damit der MS5611 im I2C-Modus arbeitet!

### HC-06 Bluetooth — Pegelwandler erforderlich

> ⚠️ Der Raspberry Pi Pico arbeitet mit **3.3V Logik**. Der HC-06 sendet auf dem TX-Pin **5V Pegel**. Ein Spannungsteiler oder Level-Shifter ist zwischen HC-06 TX und Pico RX zwingend erforderlich!

---

## Pinbelegung

Alle Pins sind zentral in `include/pins.h` definiert.

```
// Motoren
PIN_MOTOR_FL  = 11    // Front Left
PIN_MOTOR_FR  = 12    // Front Right
PIN_MOTOR_BL  = 14    // Back Left
PIN_MOTOR_BR  = 13    // Back Right

// I2C (MS5611 Barometer)
PIN_SDA       = 4
PIN_SCL       = 5

// SPI (MPU9250 + NRF24)
PIN_SPI_MOSI  = ?     // noch offen
PIN_SPI_MISO  = ?     // noch offen
PIN_SPI_SCK   = ?     // noch offen
PIN_IMU_CS    = ?     // noch offen

// Radio NRF24
PIN_RADIO_CE  = 20
PIN_RADIO_CSN = 17

// Bluetooth HC-06 (UART0)
PIN_BT_TX     = ?     // noch offen
PIN_BT_RX     = ?     // noch offen
```

---

## Projektstruktur

```
drone_pico/
├── platformio.ini          ← PlatformIO Konfiguration
├── include/                ← Alle Header-Dateien (.h)
│   ├── pins.h              ← Hardware-Pinbelegung (einzige Wahrheit!)
│   ├── config.h            ← Parameter, Konstanten, Test-Modi
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
└── src/                    ← Alle Implementierungen (.cpp)
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

> **Konvention:** `.h` Dateien in `include/`, `.cpp` Dateien in `src/`. PlatformIO findet Header automatisch über den `include/` Ordner im Projekt-Root.

---

## Bibliotheken

Alle Bibliotheken sind in `platformio.ini` definiert. **Keine neuen Bibliotheken ohne Rücksprache!**

| Bibliothek | Version | Verwendung |
|---|---|---|
| bakercp/Logger | ^1.0.3 | Logging & Debugging |
| hideakitai/TaskManager | ^0.4.8 | Kooperatives Multitasking |
| nrf24/RF24 | ^1.4.8 | NRF24L01 Funk |
| mike-matera/FastPID | ^1.3.1 | PID-Regler |
| khoih-prog/RP2040_PWM | ^1.3.0 | PWM (Fallback: nativer SDK) |
| robtillaart/MS5611 | ^0.3.9 | Barometer |
| adafruit/Adafruit Unified Sensor | ^1.1.6 | Sensor-Abstraktion |
| hideakitai/MPU9250 | ^0.4.8 | IMU |
| adafruit/Adafruit BusIO | ^1.14.1 | I2C/SPI Abstraktion |

---

## Test-Modi

Test-Modi werden in `include/config.h` per `#define` aktiviert.  
**Immer nur einen Test-Modus gleichzeitig aktiv lassen!**

```cpp
// ── Test-Modi (auskommentieren = deaktiviert) ──────────────
// #define TEST_MOTORS       // Schritt 2: ESC/Motor Test
// #define TEST_BAROMETER    // Schritt 3: MS5611 Test
#define TEST_KEYBOARD        // Schritt 4: Tastatur Test
// #define TEST_I2C_SCAN     // Diagnose: I2C Bus Scanner
```

### TEST_MOTORS
Motortest per Serial-Befehl. **Propeller abnehmen!**
- `+` → Throttle +50 µs
- `-` → Throttle -50 µs
- `s` → STOP

### TEST_BAROMETER
Barometer-Ausgabe alle 500ms: Höhe, Druck, Temperatur.

### TEST_KEYBOARD
Pfeiltasten-Erkennung + Barometer-Ausgabe kombiniert.
- Pfeil hoch → Throttle +
- Pfeil runter → Throttle -
- `r` → Barometer rekalibrieren
- `s` → Stop
- `h` → Hilfe

> ⚠️ **Pfeiltasten funktionieren nur im PlatformIO Terminal** (nicht im eingebauten Serial Monitor). PuTTY oder minicom funktionieren ebenfalls.

### TEST_I2C_SCAN
Scannt den I2C-Bus und gibt alle gefundenen Adressen aus.
- `0x68` → MPU9250 ✅
- `0x77` → MS5611 ✅

---

## ESC PWM Parameter

| Parameter | Wert |
|---|---|
| Frequenz | 50 Hz |
| Minimum (Stop) | 1000 µs |
| Maximum (Vollgas) | 2000 µs |
| Throttle-Schritt | 50 µs |

> ⚠️ ESCs beim ersten Start kalibrieren: Pico startet alle ESCs mit 1000 µs Minimalsignal für 2 Sekunden.

---

## Barometer Konfiguration

| Parameter | Wert |
|---|---|
| Oversampling | OSR_ULTRA_HIGH (höchste Genauigkeit) |
| Filter | Gleitender Mittelwert, 20 Samples |
| Kalibrierung | 20 Messungen à 100ms beim Start |
| Genauigkeit (Stillstand) | ± 0.8 cm |
| I2C Adresse | 0x77 |

---

## Bekannte Probleme & Lösungen

### RP2040_PWM Bibliothek — Kompilierungsfehler
**Problem:** `khoih-prog/RP2040_PWM` hat unter PlatformIO Probleme mit dem Header `hardware/pwm.h`.  
**Lösung:** Nativer RP2040-SDK PWM wird direkt verwendet (`hardware/pwm.h` via SDK). `RP2040_PWM` Bibliothek bleibt in `platformio.ini` aber wird im Code nicht eingebunden.

### MS5611 nicht gefunden
**Problem:** `[BARO] ERROR: MS5611 nicht gefunden!`  
**Ursachen & Lösungen:**
1. PS-Pin nicht an 3.3V → PS und NCS an 3.3V anschließen
2. SDA/SCL vertauscht → Verkabelung prüfen
3. Falsches Board → I2C-Scanner starten (`TEST_I2C_SCAN`)

### Höhenwerte unrealistisch (z.B. 29000 cm)
**Problem:** Referenzdruck weicht stark von Standardatmosphäre ab.  
**Lösung:** `calibrate()` setzt Referenzdruck automatisch auf aktuellen Messwert — Höhe ist immer relativ zum Startpunkt, nicht zum Meeresspiegel.

### Pfeiltasten werden nicht erkannt
**Problem:** Eingebauter PlatformIO Serial Monitor überträgt keine ANSI Escape-Sequenzen.  
**Lösung:** PlatformIO Terminal im VSCode Terminal-Fenster verwenden: `pio device monitor`

---

## Nächste Schritte

| Schritt | Inhalt | Status |
|---|---|---|
| 1 | Projektstruktur | ✅ Fertig |
| 2 | PWM & Motortest | ✅ Fertig |
| 3 | Barometer MS5611 | ✅ Fertig |
| 4 | Tastatur-Eingabe | 🔄 Aktuell |
| 5 | PID-Regler | ⏳ Offen |
| 6 | Bluetooth PID-Config | ⏳ Offen |
| 7 | Flash-Speicher (EEPROM) | ⏳ Offen |
| 8 | Schwebeflug-Test | ⏳ Offen |

---

## Entwicklungsregeln

- Schritt für Schritt — kein nächster Schritt bevor der aktuelle funktioniert
- Keine neuen Bibliotheken ohne Rücksprache
- Fehler zuerst beheben, dann weitermachen
- `pins.h` ist die einzige Wahrheit für alle Pin-Definitionen
- `.h` in `include/`, `.cpp` in `src/`

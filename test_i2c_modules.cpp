// ============================================================
//  I2C Modul-Test  —  Breadboard-Diagnose
//  Pico 3V3-Pin → VCC aller Module (NIEMALS VBUS/5V!)
//
//  Verwendung: Diese Datei als main.cpp in neuem PlatformIO-
//  Projekt einsetzen (gleiche platformio.ini wie Drohne).
//  SDA = GPIO4, SCL = GPIO5
// ============================================================
#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA 4
#define PIN_SCL 5

// MPU9250
#define MPU_ADDR      0x68
#define REG_WHO_AM_I  0x75
#define REG_PWR_MGMT  0x6B

// MS5611
#define MS5611_ADDR   0x77
#define MS5611_RESET  0x1E
#define MS5611_PROM   0xA0  // + 2*n  (n=0..7)

// BMP280 (auf GY-91)
#define BMP280_ADDR   0x76
#define BMP280_ID_REG 0xD0

// ── Hilfsfunktionen ─────────────────────────────────────────

uint8_t readReg(uint8_t devAddr, uint8_t reg) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom(devAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

void writeReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void separator() {
    Serial.println("--------------------------------------------");
}

// ── I2C Scan ────────────────────────────────────────────────

void i2cScan() {
    separator();
    Serial.println("I2C SCAN");
    separator();
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X gefunden\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("  Kein Geraet!");
    Serial.printf("  Gesamt: %d Geraet(e)\n", found);
}

// ── MPU9250 ─────────────────────────────────────────────────

void testMPU9250() {
    separator();
    Serial.println("MPU9250  (erwartet: 0x68)");
    separator();

    // Wakeup: PWR_MGMT_1 = 0 (aus Sleep holen)
    writeReg(MPU_ADDR, REG_PWR_MGMT, 0x00);
    delay(10);

    uint8_t who = readReg(MPU_ADDR, REG_WHO_AM_I);
    Serial.printf("  WHO_AM_I: 0x%02X  ", who);

    if (who == 0x71 || who == 0x73 || who == 0x70) {
        Serial.println("OK");
    } else if (who == 0xFF) {
        Serial.println("FEHLER — kein ACK (nicht gefunden oder 5V-Schaden!)");
    } else {
        Serial.println("UNBEKANNT — moeglicher Klon oder Defekt");
    }

    // Gyro-Roh-Wert lesen als Lebenszeichen
    uint8_t xh = readReg(MPU_ADDR, 0x43);
    uint8_t xl = readReg(MPU_ADDR, 0x44);
    if (xh != 0xFF && xl != 0xFF) {
        int16_t gx = (int16_t)(xh << 8 | xl);
        Serial.printf("  Gyro-X raw: %d  (!=0 = aktiv)\n", gx);
    }
}

// ── MS5611 ──────────────────────────────────────────────────

void testMS5611() {
    separator();
    Serial.println("MS5611  (erwartet: 0x77)");
    separator();

    // Reset senden
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_RESET);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("  FEHLER — kein ACK auf Reset (err=%d)\n", err);
        return;
    }
    delay(10);

    // PROM auslesen (6 Kalibrierkoeffizienten C1..C6)
    bool ok = true;
    Serial.print("  PROM: ");
    for (uint8_t i = 1; i <= 6; i++) {
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write(MS5611_PROM | (i * 2));
        Wire.endTransmission(false);
        Wire.requestFrom(MS5611_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            uint16_t c = ((uint16_t)Wire.read() << 8) | Wire.read();
            Serial.printf("C%d=%u ", i, c);
            if (c == 0 || c == 0xFFFF) ok = false;
        } else {
            Serial.printf("C%d=ERR ", i);
            ok = false;
        }
    }
    Serial.println();
    Serial.println(ok ? "  -> OK" : "  -> FEHLER: Nullwerte im PROM");
}

// ── BMP280 ──────────────────────────────────────────────────

void testBMP280() {
    separator();
    Serial.println("BMP280  (GY-91, erwartet: 0x76)");
    separator();

    uint8_t id = readReg(BMP280_ADDR, BMP280_ID_REG);
    Serial.printf("  Chip-ID: 0x%02X  ", id);

    if (id == 0x60)      Serial.println("BME280 OK");
    else if (id == 0x58) Serial.println("BMP280 OK");
    else if (id == 0xFF) Serial.println("FEHLER — kein ACK");
    else                 Serial.printf("UNBEKANNT\n");
}

// ── Setup / Loop ─────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    // Warten bis Monitor offen ist (max 10 s)
    uint32_t t = millis();
    while (!Serial && millis() - t < 10000) delay(10);

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(100000);
    delay(100);

    Serial.println("============================================");
    Serial.println("  I2C Modul-Test  —  Breadboard-Diagnose");
    Serial.println("  VCC = 3.3 V (Pico 3V3-Pin)!");
    Serial.println("  SDA = GPIO4  |  SCL = GPIO5");
    Serial.println("============================================");
}

void loop() {
    i2cScan();
    testMPU9250();
    testMS5611();
    testBMP280();
    Serial.println();
    delay(5000);
}

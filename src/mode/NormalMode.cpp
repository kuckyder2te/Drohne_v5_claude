#include "mode/NormalMode.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/FlightController.h"
#include "control/InputHandler.h"
#include "Barometer.h"
#include "Ultrasonic.h"
#include "IMU.h"
#include "Battery.h"
#include "comm/CommChannel.h"
#include "comm/cli.h"
#include "storage/Settings.h"

// Gemeinsame Firmware-Objekte. Hier definiert (frueher in main.cpp), da
// NormalMode die alleinige Kompositionswurzel ist; per extern genutzt von
// FlightController/InputHandler/cli/myLogger.
Barometer baro;
Battery battery;
Ultrasonic ultrasonic;
FlightController flightController;
CommChannel* comm = nullptr;   // aktiver Eingabe-/Log-Kanal, Auswahl per COMM_USE_BLUETOOTH (config.h)
Settings settings;
IMU imu;

void NormalMode::setup()
{
#ifdef COMM_USE_BLUETOOTH
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
    comm = new CommChannel(Serial1);
#else
    comm = new CommChannel(Serial);
#endif
    Serial.begin(115200);
    cli::begin(Serial);

    delay(2000);

    comm->sendLine("[BT] Drohne bereit");
    comm->sendLine("[BT] Befehle: A D R L H  +/-");
    comm->sendLine("[BT] PID: P=x I=x D=x  RP= RI= RD=  PP= PI= PD=");
    comm->sendLine("[BT] S=Speichern  RESET  ?=Abfrage");
    comm->sendLine("[CLI] Neu: ':' + Zeile fuer Shell-Befehle, z.B. :setHeight 30  (:help fuer Liste)");
    LOG("[BT] Bluetooth bereit");

    LOG("=== DROHNE PICO BOOT ====");
    LOG(">> Modus: NORMALBETRIEB");

    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();

    if (!baro.begin())
    {
        LOG("FEHLER: Barometer! Programm gestoppt.");
        while (true)
            delay(1000);
    }

    ultrasonic.begin();
    flightController.begin(settings);

    battery.begin();

    if (!imu.begin(false))
    {
        LOG("WARNUNG: IMU nicht gefunden!");
    }

    comm->printHelp();
    LOG("[CTRL] Bereit - 'a' zum Armen");
}

void NormalMode::loop()
{
    cli::update();

    baro.update();
    ultrasonic.update();
    imu.update();
    battery.update();

    // Bei IMU Fehler oder Höhensprung → sofort DISARM
    flightController.checkSafety(imu.isReady(), baro.getAltitudeCm());

    InputHandler::handle();

    // PID-Regelkreis
    flightController.updateControlLoop(ultrasonic, baro, imu);

    // Statusausgabe alle 500ms
    flightController.logStatus(battery, baro, ultrasonic);
}

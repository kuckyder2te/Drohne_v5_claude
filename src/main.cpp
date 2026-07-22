#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/FlightController.h"
#include "Barometer.h"
#include "comm/CommChannel.h"
#include "comm/cli.h"
#include "storage/Settings.h"
#include "IMU.h"
#include "Battery.h"
#include "Ultrasonic.h"
#include "testmode/TestModes.h"
#include "control/InputHandler.h"

Barometer baro;
Battery battery;
Ultrasonic ultrasonic;
FlightController flightController;

CommChannel* comm = nullptr;   // aktiver Eingabe-/Log-Kanal, Auswahl per COMM_USE_BLUETOOTH (config.h)
Settings settings;
IMU imu;

// -- Setup --------------------------------------------------
void setup()
{
#ifdef COMM_USE_BLUETOOTH
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    Serial1.begin(BT_BAUD);
    comm = new CommChannel(Serial1);

#else
    Serial.begin(115200);
    comm = new CommChannel(Serial);
    cli::begin(Serial);
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

    battery.begin();
    ultrasonic.begin();

    LOG("=== DROHNE PICO BOOT ====");

    TestModes::setup();

#ifdef NORMALBETRIEB
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

#endif // NORMALBETRIEB
}

// -- Loop ---------------------------------------------------
void loop()
{
    cli::update();
    battery.update();

    TestModes::loop();

    // -- NORMALBETRIEB --------------------------------------
#ifdef NORMALBETRIEB

    baro.update();
    ultrasonic.update();
    imu.update();

    // Bei IMU Fehler oder Höhensprung → sofort DISARM
    flightController.checkSafety(imu.isReady(), baro.getAltitudeCm());

    InputHandler::handle();

    // PID-Regelkreis
    flightController.updateControlLoop(ultrasonic, baro, imu);

    // Statusausgabe alle 500ms
    flightController.logStatus(battery, baro, ultrasonic);

#endif // NORMALBETRIEB
}

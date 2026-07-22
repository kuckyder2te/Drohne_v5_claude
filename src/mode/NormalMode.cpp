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
#include "storage/Settings.h"

// Zugriff auf die main.cpp-Globalen, wie in InputHandler.cpp/KeyboardTestMode.cpp.
extern Barometer baro;
extern Ultrasonic ultrasonic;
extern IMU imu;
extern Battery battery;
extern FlightController flightController;
extern CommChannel* comm;
extern Settings settings;

void NormalMode::setup()
{
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
}

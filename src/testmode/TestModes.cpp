#include "testmode/TestModes.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "pins.h"
#include "control/FlightController.h"
#include "Barometer.h"
#include "comm/CommChannel.h"
#include "storage/Settings.h"

// Zugriff auf die main.cpp-Globalen wie in src/comm/cli.cpp
// (extern float targetHeightCm;) / src/myLogger.cpp ueber extern-Globale.
extern Barometer baro;
extern FlightController flightController;
extern CommChannel* comm;
extern Settings settings;

void TestModes::setup()
{
#ifdef TEST_KEYBOARD
    LOG(">> Modus: KEYBOARD TEST");
    if (!baro.begin())
    {
        LOG("FEHLER: Barometer nicht gefunden.");
        while (true)
            delay(1000);
    }
    comm->printHelp();
#endif
}

void TestModes::loop()
{
    // -- TEST_KEYBOARD --------------------------------------
#ifdef TEST_KEYBOARD
    baro.update();
    KeyEvent key = comm->getKey();
    String tkCmd = comm->getCommand();
    if (tkCmd.length() > 0)
        comm->processCommand(tkCmd, flightController.getPidHeight(), flightController.getPidRoll(), flightController.getPidPitch(), settings);
    switch (key)
    {
    case KeyEvent::ARROW_UP:
        LOG("[KEY] Pfeil HOCH");
        break;
    case KeyEvent::ARROW_DOWN:
        LOG("[KEY] Pfeil RUNTER");
        break;
    case KeyEvent::KEY_D:
        LOG("[KEY] DISARM");
        break;
    case KeyEvent::KEY_R:
        baro.calibrate();
        break;
    case KeyEvent::KEY_H:
        comm->printHelp();
        break;
    case KeyEvent::KEY_A:
        LOG("[KEY] ARM - nur im Normalbetrieb");
        break;
    case KeyEvent::NONE:
        break;
    }
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();
        LOG_FMT("[BARO] Hoehe: %.1f cm", baro.getAltitudeCm());
    }
#endif
}

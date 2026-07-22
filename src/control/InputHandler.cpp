#include "control/InputHandler.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "comm/CommChannel.h"
#include "control/FlightController.h"
#include "Barometer.h"
#include "IMU.h"
#include "storage/Settings.h"

// Zugriff auf die main.cpp-Globalen wie in src/comm/cli.cpp
// (extern float targetHeightCm;) / src/testmode/TestModes.cpp ueber extern-Globale.
extern CommChannel* comm;
extern FlightController flightController;
extern Settings settings;
extern Barometer baro;
extern IMU imu;

void InputHandler::handle()
{
    // ARM-Bestätigung Timeout
    flightController.updateArmPendingTimeout();

    // Eingabe ueber den aktiven Kanal (comm; Auswahl per COMM_USE_BLUETOOTH in config.h)
    KeyEvent key = comm->getKey();
    String   cmd = comm->getCommand();
        if (cmd.length() > 0)
            comm->processCommand(cmd, flightController.getPidHeight(), flightController.getPidRoll(), flightController.getPidPitch(), settings);

    switch (key)
    {
    case KeyEvent::ARROW_UP:
        flightController.adjustTargetHeight(10.0f);
        break;

    case KeyEvent::ARROW_DOWN:
        flightController.adjustTargetHeight(-10.0f);
        break;

    case KeyEvent::KEY_A:
        flightController.requestArm(imu.isReady(), baro);
        break;

    case KeyEvent::KEY_D:
        flightController.disarm();
        break;

    case KeyEvent::KEY_R:
        flightController.recalibrate(baro);
        break;

    case KeyEvent::KEY_H:
        comm->printHelp();
        break;

    case KeyEvent::KEY_L:
        flightController.toggleStatusLog();
        break;

    case KeyEvent::NONE:
        break;
    }
}

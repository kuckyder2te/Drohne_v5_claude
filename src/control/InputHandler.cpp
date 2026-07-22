#include "control/InputHandler.h"

#include <Arduino.h>
#include "myLogger.h"
#include "config.h"
#include "comm/CommChannel.h"
#include "control/PIDController.h"
#include "Barometer.h"
#include "IMU.h"
#include "storage/Settings.h"

// Zugriff auf die main.cpp-Globalen wie in src/comm/cli.cpp
// (extern float targetHeightCm;) / src/testmode/TestModes.cpp ueber extern-Globale.
extern CommChannel* comm;
extern PIDController pidHeight;
extern PIDController pidRoll;
extern PIDController pidPitch;
extern Settings settings;
extern Barometer baro;
extern IMU imu;

extern float targetHeightCm;
extern bool armed;
extern bool statusLogEnabled;
extern bool armPending;
extern uint32_t armPendingMs;
extern uint32_t lastPidMs;

void printHelp(); // bleibt in main.cpp (auch vom Normalbetrieb genutzt)
void disarm();    // bleibt in main.cpp (auch vom Sicherheits-Check genutzt)

void InputHandler::handle()
{
    // ARM-Bestätigung Timeout
    if (armPending && (millis() - armPendingMs > 3000))
    {
        armPending = false;
        LOG("[CTRL] ARM abgebrochen (Timeout)");
    }

    // Eingabe ueber den aktiven Kanal (comm; Auswahl per COMM_USE_BLUETOOTH in config.h)
    KeyEvent key = comm->getKey();
    String   cmd = comm->getCommand();
        if (cmd.length() > 0)
            comm->processCommand(cmd, pidHeight, pidRoll, pidPitch, settings);

    switch (key)
    {
    case KeyEvent::ARROW_UP:
        targetHeightCm = constrain(targetHeightCm + 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
        LOG_FMT("[CTRL] Zielhoehe: %.1f cm", targetHeightCm);
        break;

    case KeyEvent::ARROW_DOWN:
        targetHeightCm = constrain(targetHeightCm - 10.0f, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
        LOG_FMT("[CTRL] Zielhoehe: %.1f cm", targetHeightCm);
        break;

    case KeyEvent::KEY_A:
        if (!armed)
        {
            if (!armPending)
            {
                armPending = true;
                armPendingMs = millis();
                LOG("[CTRL] ARM? Nochmal 'a' druecken (3s)");
            }
            else if (millis() - armPendingMs <= 3000)
            {
                armPending = false;
                if (!imu.isReady())
                {
                    LOG("[CTRL] ARM verweigert - IMU nicht bereit!");
                }
                else
                {
                    LOG("[CTRL] Rekalibrierung vor ARM...");
                    baro.calibrate();
                    delay(500);
                    armed = true;
                    targetHeightCm = 20.0f;
                    lastPidMs = millis();
                    pidHeight.reset();
                    pidRoll.reset();
                    pidPitch.reset();
                    LOG("[CTRL] ARM - Ziel: 20 cm");
                }
            }
            else
            {
                armPending = false;
                LOG("[CTRL] ARM abgebrochen (Timeout)");
            }
        }
        break;

    case KeyEvent::KEY_D:
        disarm();
        break;

    case KeyEvent::KEY_R:
        if (!armed)
        {
            baro.calibrate();
            pidHeight.reset();
            LOG("[CTRL] Barometer rekalibriert");
        }
        else
        {
            LOG("[CTRL] Rekalibrierung nur im DISARM Modus!");
        }
        break;

    case KeyEvent::KEY_H:
        printHelp();
        break;

    case KeyEvent::KEY_L:
        statusLogEnabled = !statusLogEnabled;
        LOG_FMT("[CTRL] Statusausgabe: %s", statusLogEnabled ? "EIN" : "AUS");
        break;

    case KeyEvent::NONE:
        break;
    }
}

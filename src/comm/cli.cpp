#include "comm/cli.h"
#include "config.h"
#include "myLogger.h"
#include <SimpleSerialShell.h>

// Reine C-Funktionszeiger (SimpleSerialShell::CommandFunction) koennen
// keinen Zustand einfangen - Zugriff auf das main.cpp-Ziel daher wie in
// myLogger.cpp (extern CommChannel* comm;) ueber ein extern-Global.
extern float targetHeightCm;

namespace {
    Stream  *cliStream = nullptr;
    bool     capturing = false;
    uint32_t lastByteMs = 0;
    constexpr uint32_t CAPTURE_IDLE_TIMEOUT_MS = 5000;

    // setX/getX-Muster fuer alle CLI-Befehle, analog zu
    // PIDController::setKp()/getKp() - kuenftige Befehle (z.B.
    // setKpHeight/getKpHeight, setArmed/getArmed) folgen demselben Schema.

    int cmdSetHeight(int argc, char **argv) {
        if (argc < 2) {
            shell.println(F("usage: setHeight <cm>"));
            return -1;
        }
        float v = strtof(argv[1], nullptr);
        targetHeightCm = constrain(v, THROTTLE_MIN_CM, MAX_HEIGHT_CM);
        LOG_FMT("[CTRL] Zielhoehe: %.1f cm", targetHeightCm);
        return 0;
    }

    int cmdGetHeight(int /*argc*/, char ** /*argv*/) {
        shell.print(F("targetHeightCm="));
        shell.println(targetHeightCm);
        return 0;
    }
}

namespace cli {
    void begin(Stream &stream) {
        cliStream = &stream;
        shell.addCommand(F("setHeight cm - Zielhoehe setzen"), cmdSetHeight);
        shell.addCommand(F("getHeight - Zielhoehe ausgeben"), cmdGetHeight);
        shell.attach(stream);
    }

    bool update() {
        shell.executeIfInput();
        // if (!cliStream) return false;

        // if (!capturing) {
        //     if (cliStream->available() && cliStream->peek() == ':') {
        //         cliStream->read(); // nur den Marker konsumieren, Rest gehoert der Shell
        //         capturing = true;
        //         lastByteMs = millis();
        //     } else {
        //         return false; // keine CLI-Zeile -> CommChannel darf lesen
        //     }
        // }

        // if (cliStream->available()) lastByteMs = millis();

        // if (shell.executeIfInput()) {
        //     capturing = false; // Zeile dispatcht -> zurueck zum CommChannel-Pfad
        // } else if (millis() - lastByteMs > CAPTURE_IDLE_TIMEOUT_MS) {
        //     shell.resetBuffer(); // abgebrochene Zeile -> nicht dauerhaft blockieren
        //     capturing = false;
        // }
        return true;
    }
}

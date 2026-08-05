#include "comm/cli.h"
#include "config.h"
#include "myLogger.h"
#include "control/FlightController.h"
#include "control/SharedState.h"
#include "control/Recorder.h"
#include "storage/Settings.h"
#include "Barometer.h"
#include "IMU.h"
#include <SimpleSerialShell.h>
#include <math.h>

// Reine C-Funktionszeiger (SimpleSerialShell::CommandFunction) koennen keinen
// Zustand einfangen - Zugriff auf die in src/mode/NormalMode.cpp definierten
// Globalen daher ueber extern.
extern FlightController flightController;
extern Settings         settings;
extern Barometer        baro;
extern IMU              imu;

namespace {
    Stream *cliStream = nullptr;

    // Not-Aus-Tasten ('d', '+', '-') werden in update() aus dem Stream gefischt,
    // BEVOR die Shell sie sieht - aber nur am Zeilenanfang, sonst verschwaende
    // das '-' in "setHeight -10" als Sofortbefehl.
    bool     atLineStart = true;
    uint32_t lastByteMs  = 0;
    constexpr uint32_t IDLE_RESET_MS = 5000;

    // Quittungen laufen ueber shell.print*(), damit sie immer auf dem Kanal
    // landen, von dem der Befehl kam. LOGGER_NOTICE() geht daneben unabhaengig davon
    // an Serial und/oder BT_UART (Flags _SERIAL_LOG/_BT_LOG in config.h).

    // Parst eine Zahl, Komma wie Punkt als Dezimaltrenner (deutsche Tastatur).
    //
    // Gibt false zurueck, wenn der String keine vollstaendig verwertbare Zahl
    // ist. Das ist der eigentliche Zweck: strtof allein liefert fuer "abc"
    // still 0.0f, ein vertipptes "pid -height -Kp o.5" wuerde den Beiwert
    // also unbemerkt auf null setzen statt sich zu beschweren.
    bool parseFloatDe(const char *s, float &out) {
        if (!s || !*s) return false;

        char buf[24];
        if (strlen(s) >= sizeof(buf)) return false;  // sonst wuerde gekuerzt
        strcpy(buf, s);
        for (char *p = buf; *p; ++p)
            if (*p == ',') *p = '.';

        char *end = nullptr;
        float v = strtof(buf, &end);

        if (end == buf) return false;            // keine Ziffer gefunden
        while (*end == ' ' || *end == '\t') ++end;
        if (*end != '\0') return false;          // Rest wie "12abc" oder "1.2.3"
        if (!isfinite(v)) return false;          // "nan", "inf", Ueberlauf

        out = v;
        return true;
    }

    // ── Achsen fuer das "pid"-Kommando ─────────────────────────
    enum { AX_HEIGHT = 0, AX_ROLL, AX_PITCH, AX_COUNT };

    const char *axisName(int ax) {
        switch (ax) {
            case AX_HEIGHT: return "height";
            case AX_ROLL:   return "roll";
            default:        return "pitch";
        }
    }

    PIDController &axisPid(int ax) {
        switch (ax) {
            case AX_HEIGHT: return flightController.getPidHeight();
            case AX_ROLL:   return flightController.getPidRoll();
            default:        return flightController.getPidPitch();
        }
    }

    // Optionen case-insensitiv vergleichen: '-Kd', '-kd' und '-KD' sind
    // gleichwertig, wie auch die Kommandonamen der Shell selbst.
    bool optIs(const char *arg, const char *name) {
        return strcasecmp(arg, name) == 0;
    }

    // Standardwerte je Achse aus config.h - Ziel von "pid -reset".
    PidCoeffs axisDefaults(int ax) {
        switch (ax) {
            case AX_HEIGHT: return {PID_KP_HEIGHT, PID_KI_HEIGHT, PID_KD_HEIGHT};
            case AX_ROLL:   return {PID_KP_ROLL,   PID_KI_ROLL,   PID_KD_ROLL};
            default:        return {PID_KP_PITCH,  PID_KI_PITCH,  PID_KD_PITCH};
        }
    }

    // Ein Achsen-Objekt als JSON-Fragment, ohne umschliessende Klammern.
    void printAxisJson(int ax) {
        PIDController &p = axisPid(ax);
        shell.print('"');
        shell.print(axisName(ax));
        shell.print(F("\":{\"Kp\":"));  shell.print(p.getKp(), 4);
        shell.print(F(",\"Ki\":"));     shell.print(p.getKi(), 4);
        shell.print(F(",\"Kd\":"));     shell.print(p.getKd(), 4);
        shell.print('}');
    }

    // Gibt die markierten Achsen als JSON-Objekt aus; mask == nullptr -> alle.
    void printPidJson(const bool *mask) {
        shell.print('{');
        bool first = true;
        for (int ax = 0; ax < AX_COUNT; ++ax) {
            if (mask && !mask[ax]) continue;
            if (!first) shell.print(',');
            printAxisJson(ax);
            first = false;
        }
        shell.println('}');
    }

    void printTargetHeight() {
        shell.print(F("targetHeightCm="));
        shell.println(flightController.getTargetHeightCm());
    }

    // ── Aktionen (Verben) ──────────────────────────────────────

    int cmdArm(int /*argc*/, char ** /*argv*/) {
        bool wasArmed = flightController.isArmed();
        flightController.requestArm(imu.isReady(), baro);

        if (flightController.isArmed())
            shell.println(wasArmed ? F("[CLI] bereits ARMED")
                                   : F("[CLI] ARMED - Ziel 20 cm"));
        else if (!imu.isReady())
            shell.println(F("[CLI] ARM verweigert - IMU nicht bereit"));
        else
            shell.println(F("[CLI] ARM? 'arm' nochmal innerhalb 3 s"));
        return 0;
    }

    int cmdStop(int /*argc*/, char ** /*argv*/) {
        flightController.disarm();
        shell.println(F("[CLI] DISARM - Motoren gestoppt"));
        return 0;
    }

    int cmdRecalibrate(int /*argc*/, char ** /*argv*/) {
        if (flightController.isArmed()) {
            shell.println(F("[CLI] Rekalibrierung nur im DISARM Modus"));
            return -1;
        }
        flightController.recalibrate(baro);
        shell.println(F("[CLI] Barometer rekalibriert"));
        return 0;
    }

    int cmdStatusLog(int /*argc*/, char ** /*argv*/) {
        flightController.toggleStatusLog();
        // Der Statuslog selbst laeuft per LOGGER_NOTICE() nach BT, nicht hierher.
        shell.println(F("[CLI] Statuslog umgeschaltet (Ausgabe laeuft ueber BT)"));
        return 0;
    }

    // Ueberschattet das eingebaute "help" der Bibliothek: addCommand() haengt
    // bei Namensgleichheit VOR den bestehenden Eintrag, und execute() nimmt
    // den ersten Treffer. So laesst sich die generierte Kommandoliste
    // weiterverwenden und um die Sofort-Tasten ergaenzen, die in ihr gar
    // nicht auftauchen koennen - sie sind keine Shell-Kommandos.
    int cmdHelp(int argc, char **argv) {
        SimpleSerialShell::printHelp(argc, argv);

        shell.println();
        shell.println(F("Sofort-Tasten (einzelne Taste, ohne Enter):"));
        shell.println(F("  d   DISARM - Motoren sofort auf Minimum ('D' ebenso)"));
        shell.println(F("  +   Zielhoehe um 10 cm erhoehen"));
        shell.println(F("  -   Zielhoehe um 10 cm verringern"));
        shell.println();
        shell.println(F("Diese drei Zeichen werden vor der Shell aus dem Datenstrom"));
        shell.println(F("gefischt und wirken daher schon beim Tastendruck, ohne Enter"));
        shell.println(F("und ohne Zeilenende. Gedacht ist das als Not-Aus, der im"));
        shell.println(F("Ernstfall nicht erst eine Zeile zu Ende getippt haben will."));
        shell.println(F("Mehrfach druecken addiert sich: '+++' sind +30 cm."));
        shell.println();
        shell.println(F("Sie gelten NUR am Zeilenanfang. Mitten in einer Eingabe sind"));
        shell.println(F("es gewoehnliche Zeichen - sonst wuerde 'setHeight -10' sein"));
        shell.println(F("Minus verlieren. Wurde eine Zeile angefangen und nicht"));
        shell.println(F("abgeschickt, verwirft die CLI sie nach 5 s ohne weitere"));
        shell.println(F("Eingabe, damit der Not-Aus wieder scharf ist."));
        shell.println(F("Aus demselben Grund faengt kein Kommandoname mit 'd' an;"));
        shell.println(F("DISARM heisst als Kommando deshalb 'stop'."));
        shell.println();
        shell.println(F("PID-Werte am Pruefstand automatisch ermitteln:"));
        shell.println(F("  1. Drohne auf der 1-Achsen-Wippe festsetzen"));
        shell.println(F("  2. 'stats' - loopHz muss ~400 sein"));
        shell.println(F("  3. 'stats -hang' - muss [SAFETY] ausloesen (ohne Propeller!)"));
        shell.println(F("  4. 'arm', dann 'tune -roll -go' (2x zum Bestaetigen)"));
        shell.println(F("  5. 'tune -show', bei Bedarf 'tune -apply'"));
        shell.println(F("  6. 'bench -dump' liefert den Messschrieb als CSV"));
        shell.println();
        shell.println(F("Optionen einzelner Kommandos: 'pid -h', 'bench -h', 'tune -help'"));
        return 0;
    }

    // ── Werte (setX/getX) ──────────────────────────────────────

    int cmdSetHeight(int argc, char **argv) {
        if (argc < 2) {
            shell.println(F("usage: setHeight <cm>"));
            return -1;
        }
        float cm;
        if (!parseFloatDe(argv[1], cm)) {
            shell.print(F("setHeight: keine gueltige Zahl: "));
            shell.println(argv[1]);
            return -1;
        }
        flightController.setTargetHeightCm(cm);
        printTargetHeight();
        return 0;
    }

    int cmdGetHeight(int /*argc*/, char ** /*argv*/) {
        printTargetHeight();
        return 0;
    }

    int cmdGetArmed(int /*argc*/, char ** /*argv*/) {
        shell.print(F("armed="));
        shell.println(flightController.isArmed() ? F("1") : F("0"));
        return 0;
    }

    // ── Zustand des Lageregelkreises auf Kern 1 ────────────────────────

    int cmdStats(int argc, char **argv) {
        // 'stats -hang' laesst Kern 1 absichtlich stehen. Das ist der
        // Nachweis, dass der Watchdog auf Kern 0 anschlaegt - ohne diesen
        // bestandenen Test darf kein Propeller montiert werden.
        if (argc >= 2 && optIs(argv[1], "-hang")) {
            if (flightController.isArmed()) {
                shell.println(F("stats: -hang nur im DISARM Modus"));
                return -1;
            }
            shell.println(F("[CLI] Kern 1 wird 1 s blockiert - erwartet: [SAFETY] Meldung"));
            shared::g_hangTest = true;
            delay(1000);
            shared::g_hangTest = false;
            return 0;
        }

        shared::CoreTlm t;
        shared::tlmRead(t);

        shell.print(F("{\"loopHz\":"));    shell.print(t.loopHz);
        shell.print(F(",\"maxLoopUs\":")); shell.print(t.maxLoopUs);
        shell.print(F(",\"overruns\":"));  shell.print(t.overruns);
        shell.print(F(",\"imuReady\":"));  shell.print(t.imuReady ? 1 : 0);
        shell.print(F(",\"running\":"));   shell.print(t.running ? 1 : 0);
        shell.print(F(",\"roll\":"));      shell.print(t.roll, 2);
        shell.print(F(",\"pitch\":"));     shell.print(t.pitch, 2);
        shell.print(F(",\"gRoll\":"));     shell.print(t.gyroRoll, 1);
        shell.print(F(",\"gPitch\":"));    shell.print(t.gyroPitch, 1);
        shell.print(F(",\"rollOut\":"));   shell.print(t.rollOut, 1);
        shell.print(F(",\"pitchOut\":"));  shell.print(t.pitchOut, 1);
        shell.print(F(",\"m\":["));
        for (uint8_t i = 0; i < 4; ++i) {
            if (i) shell.print(',');
            shell.print(t.m[i]);
        }
        shell.print(F("],\"beatC1\":"));   shell.print((unsigned long)shared::g_beatC1);
        shell.print(F(",\"rec\":"));       shell.print(recorder::count());
        shell.println('}');
        return 0;
    }

    // Sicherung aller drei Regler. Von 'pid -save' und 'tune -save' benutzt,
    // damit beide exakt dieselbe Wirkung haben.
    void saveAllPid() {
        PidCoeffs c[AX_COUNT];
        for (int ax = 0; ax < AX_COUNT; ++ax) {
            PIDController &p = axisPid(ax);
            c[ax] = {p.getKp(), p.getKi(), p.getKd()};
        }
        settings.save(c[AX_HEIGHT], c[AX_ROLL], c[AX_PITCH]);
    }

    void pidHelp() {
        shell.println(F("pid - PID-Koeffizienten anzeigen, setzen, sichern"));
        shell.println(F("usage: pid [-reset] [-height|-roll|-pitch] [-Kp|-Ki|-Kd <w>] [-save]"));
        shell.println(F("  pid                       alle Regler als JSON"));
        shell.println(F("  pid -height               nur den Hoehenregler als JSON"));
        shell.println(F("  pid -height -Kd 10        Kd des Hoehenreglers setzen"));
        shell.println(F("  pid -roll -Kp 1,2 -Ki 0.1 mehrere Koeffizienten auf einmal"));
        shell.println(F("  pid -height -Kp 2 -roll -Kp 1   mehrere Achsen je Aufruf"));
        shell.println(F("  pid -save                 alle drei Regler ins EEPROM"));
        shell.println(F("  pid -reset                alle drei auf Standardwerte"));
        shell.println(F("  pid -height -Kp 2 -save   aendern und sichern in einem Zug"));
        shell.println(F("Die Achse gilt fuer alle folgenden -K-Optionen und muss vor"));
        shell.println(F("ihnen stehen. Optionen sind case-insensitiv, Komma als"));
        shell.println(F("Dezimaltrenner ist erlaubt. Ausgegeben werden die genannten"));
        shell.println(F("Achsen - ohne Achsenangabe alle."));
        shell.println(F("Ein Wert, der keine gueltige Zahl ist, bricht den ganzen"));
        shell.println(F("Aufruf ab - es wird dann nichts gesetzt, sichert und"));
        shell.println(F("zurueckgesetzt auch nicht."));
        shell.println(F("-save/-reset gelten IMMER fuer alle drei Regler (das EEPROM"));
        shell.println(F("hat nur einen Gueltigkeitsmarker) und werden unabhaengig von"));
        shell.println(F("ihrer Position ausgefuehrt: erst -reset, dann die -K-Werte,"));
        shell.println(F("zuletzt -save."));
    }

    // Loest die frueheren neun setK*-Kommandos sowie getPid, save und reset
    // ab. Die Achse ist zustandsbehaftet: sie gilt fuer alle nachfolgenden
    // -K-Optionen, daher laesst sich in einem Aufruf mehr als ein Regler
    // stellen.
    int cmdPid(int argc, char **argv) {
        if (argc >= 2 && optIs(argv[1], "-h")) {
            pidHelp();
            return 0;
        }

        // Durchgang 1: -reset/-save nur einsammeln. Beide wirken auf alle
        // Achsen und werden in fester Reihenfolge ausgefuehrt (reset -> Werte
        // -> save), damit "pid -save -height -Kp 2" nicht den Stand VOR der
        // Aenderung sichert - die Position im Aufruf soll egal sein.
        bool doReset = false;
        bool doSave  = false;
        for (int i = 1; i < argc; ++i) {
            if      (optIs(argv[i], "-reset")) doReset = true;
            else if (optIs(argv[i], "-save"))  doSave  = true;
        }

        // EEPROM.commit() ruft rp2040.idleOtherCore() und haelt damit den
        // Lageregelkreis auf Kern 1 fuer die gesamte Flash-Loeschung an -
        // die Motoren stuenden solange auf ihrem letzten Wert.
        if (doSave && flightController.isArmed()) {
            shell.println(F("pid: -save nur im DISARM Modus (haelt Kern 1 an)"));
            return -1;
        }

        // Durchgang 2: alles pruefen und die Schreibvorgaenge nur SAMMELN.
        // Angewendet wird erst in Durchgang 3, wenn die ganze Zeile fehlerfrei
        // ist - sonst hinterliesse ein Tippfehler im dritten Argument die
        // ersten beiden bereits gesetzt, und man taete beim Tuning mit einem
        // halb geaenderten Regler weiter.
        struct PidWrite { int8_t axis; char which; float value; };
        PidWrite writes[8];
        uint8_t  nWrites = 0;

        bool selected[AX_COUNT] = {false, false, false};
        bool anySelected = false;
        int  current     = -1;   // aktuelle Achse, -1 = noch keine gewaehlt

        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i];

            if (optIs(a, "-reset") || optIs(a, "-save")) continue;  // s.o.

            int ax = -1;
            if      (optIs(a, "-height")) ax = AX_HEIGHT;
            else if (optIs(a, "-roll"))   ax = AX_ROLL;
            else if (optIs(a, "-pitch"))  ax = AX_PITCH;

            if (ax >= 0) {
                current = ax;
                selected[ax] = true;
                anySelected  = true;
                continue;
            }

            bool isKp = optIs(a, "-Kp");
            bool isKi = optIs(a, "-Ki");
            bool isKd = optIs(a, "-Kd");
            if (isKp || isKi || isKd) {
                // Bewusst keine Vorgabe-Achse: sonst landet ein vergessenes
                // -height still im Hoehenregler statt im gemeinten.
                if (current < 0) {
                    shell.println(F("pid: erst Achse waehlen (-height, -roll oder -pitch)"));
                    return -1;
                }
                if (i + 1 >= argc) {
                    shell.print(F("pid: Wert fehlt nach "));
                    shell.println(a);
                    return -1;
                }
                const char *valStr = argv[++i];
                float v;
                if (!parseFloatDe(valStr, v)) {
                    shell.print(F("pid: keine gueltige Zahl nach "));
                    shell.print(a);
                    shell.print(F(": "));
                    shell.println(valStr);
                    return -1;
                }
                if (nWrites >= (uint8_t)(sizeof(writes) / sizeof(writes[0]))) {
                    shell.println(F("pid: zu viele Werte in einem Aufruf"));
                    return -1;
                }
                writes[nWrites].axis  = (int8_t)current;
                writes[nWrites].which = isKp ? 'P' : (isKi ? 'I' : 'D');
                writes[nWrites].value = v;
                ++nWrites;
                continue;
            }

            shell.print(F("pid: unbekannte Option "));
            shell.println(a);
            shell.println(F("pid -h fuer Hilfe"));
            return -1;
        }

        // Durchgang 3: ab hier kann nichts mehr fehlschlagen
        if (doReset) {
            settings.reset();
            for (int ax = 0; ax < AX_COUNT; ++ax) {
                PidCoeffs d = axisDefaults(ax);
                PIDController &p = axisPid(ax);
                p.setKp(d.kp);
                p.setKi(d.ki);
                p.setKd(d.kd);
            }
        }

        for (uint8_t w = 0; w < nWrites; ++w) {
            PIDController &p = axisPid(writes[w].axis);
            switch (writes[w].which) {
                case 'P': p.setKp(writes[w].value); break;
                case 'I': p.setKi(writes[w].value); break;
                default:  p.setKd(writes[w].value); break;
            }
        }

        if (doSave) saveAllPid();

        // Geaenderte Koeffizienten an den Lageregelkreis auf Kern 1 geben -
        // _pidRoll/_pidPitch hier sind seit dem Dual-Core-Umbau nur noch
        // Koeffizientenspeicher und rechnen selbst nichts mehr.
        flightController.publish();

        // Nach -save/-reset immer alle Achsen zeigen: beide betreffen alle
        // drei, eine auf eine Achse eingeschraenkte Ausgabe waere irrefuehrend.
        printPidJson((anySelected && !doSave && !doReset) ? selected : nullptr);
        return 0;
    }

    // ── Pruefstand (1-Achsen-Wippe) ────────────────────────────────────

    void benchHelp() {
        shell.println(F("bench - Pruefstandsbetrieb auf genau EINER Achse"));
        shell.println(F("usage: bench [-roll|-pitch] [-throttle us] [-max us]"));
        shell.println(F("             [-limit deg] [-time s] [-go|-stop|-dump]"));
        shell.println(F("  -roll/-pitch  aktive Achse, die andere bleibt bei 0"));
        shell.println(F("  -throttle us  feste Basis-Throttle (Vorgabe 1300)"));
        shell.println(F("  -max us       Obergrenze aller Motoren (Vorgabe 1600)"));
        shell.println(F("  -limit deg    Abbruch ab diesem Winkel (Vorgabe 25)"));
        shell.println(F("  -time s       Laufzeitbegrenzung (Vorgabe 20)"));
        shell.println(F("  -go           starten, 2x innerhalb 3 s wie bei 'arm'"));
        shell.println(F("  -stop         beenden, Motoren auf Minimum"));
        shell.println(F("  -dump [-n k]  Aufzeichnung als CSV (nur disarmt)"));
        shell.println(F("Der Hoehenregler ist hier abgeschaltet, die Throttle ist"));
        shell.println(F("fest. Aufgezeichnet wird ab -go automatisch."));
        shell.println(F("Vor dem ersten Lauf mit Propeller: 'stats -hang' machen."));
    }

    // Gemeinsamer Parser fuer bench/tune. Beide teilen Achse, Throttle,
    // Grenzen und die Start-/Stop-Optionen; getrennt sind nur die
    // Relais-Parameter und die Auswertung.
    struct RunOpts {
        uint8_t  axis      = shared::AXIS_ROLL;
        bool     axisSet   = false;
        float    throttle  = BENCH_THROTTLE_US;
        float    maxUs     = BENCH_MAX_US;
        float    limitDeg  = BENCH_LIMIT_DEG;
        uint32_t timeoutS  = BENCH_TIMEOUT_S;
        bool     go = false, stop = false, dump = false;
        uint16_t lastN = 0;
    };

    // Zweistufige Bestaetigung wie bei requestArm: ein Tippfehler soll die
    // Motoren nicht anwerfen.
    uint32_t g_benchPendingMs = 0;
    uint32_t g_tunePendingMs  = 0;

    bool confirmStart(uint32_t &pendingMs, const __FlashStringHelper *what) {
        uint32_t now = millis();
        if (pendingMs && (now - pendingMs) <= 3000) {
            pendingMs = 0;
            return true;
        }
        pendingMs = now;
        shell.print(F("[CLI] "));
        shell.print(what);
        shell.println(F(" startet Motoren! Zur Bestaetigung erneut -go (3 s)"));
        return false;
    }

    // Liest die gemeinsamen Optionen. Rueckgabe false = Fehler, bereits gemeldet.
    // 'unknown' bekommt eine Option, die der Aufrufer selbst deuten soll.
    bool parseRunOpt(const char *a, int &i, int argc, char **argv,
                     RunOpts &o, bool &handled) {
        handled = true;
        if      (optIs(a, "-roll"))  { o.axis = shared::AXIS_ROLL;  o.axisSet = true; return true; }
        else if (optIs(a, "-pitch")) { o.axis = shared::AXIS_PITCH; o.axisSet = true; return true; }
        else if (optIs(a, "-go"))    { o.go   = true; return true; }
        else if (optIs(a, "-stop"))  { o.stop = true; return true; }
        else if (optIs(a, "-dump"))  { o.dump = true; return true; }

        const bool isThr   = optIs(a, "-throttle");
        const bool isMax   = optIs(a, "-max");
        const bool isLimit = optIs(a, "-limit");
        const bool isTime  = optIs(a, "-time");
        const bool isN     = optIs(a, "-n");
        if (isThr || isMax || isLimit || isTime || isN) {
            if (i + 1 >= argc) {
                shell.print(F("Wert fehlt nach ")); shell.println(a);
                return false;
            }
            float v;
            if (!parseFloatDe(argv[++i], v)) {
                shell.print(F("keine gueltige Zahl nach ")); shell.println(a);
                return false;
            }
            if      (isThr)   o.throttle = v;
            else if (isMax)   o.maxUs    = v;
            else if (isLimit) o.limitDeg = v;
            else if (isTime)  o.timeoutS = (uint32_t)v;
            else              o.lastN    = (uint16_t)v;
            return true;
        }

        handled = false;
        return true;
    }

    // Prueft die Werte, die die Motoren betreffen. Getrennt von der
    // Syntaxpruefung, damit ein unsinniger Wert nie bis zum Start kommt.
    bool validateRun(const RunOpts &o) {
        if (o.throttle < ESC_MIN_US || o.throttle > ESC_MAX_US) {
            shell.println(F("throttle ausserhalb 1000..2000"));
            return false;
        }
        if (o.maxUs < ESC_MIN_US || o.maxUs > ESC_MAX_US) {
            shell.println(F("max ausserhalb 1000..2000"));
            return false;
        }
        if (o.maxUs < o.throttle) {
            shell.println(F("max liegt unter throttle - so kann nichts regeln"));
            return false;
        }
        if (o.limitDeg <= 1.0f || o.limitDeg > 80.0f) {
            shell.println(F("limit ausserhalb 1..80 Grad"));
            return false;
        }
        if (o.timeoutS == 0 || o.timeoutS > 120) {
            shell.println(F("time ausserhalb 1..120 s"));
            return false;
        }
        return true;
    }

    int cmdBench(int argc, char **argv) {
        if (argc >= 2 && (optIs(argv[1], "-h") || optIs(argv[1], "-help"))) {
            benchHelp();
            return 0;
        }

        RunOpts o;
        for (int i = 1; i < argc; ++i) {
            bool handled;
            if (!parseRunOpt(argv[i], i, argc, argv, o, handled)) return -1;
            if (!handled) {
                shell.print(F("bench: unbekannte Option ")); shell.println(argv[i]);
                shell.println(F("bench -h fuer Hilfe"));
                return -1;
            }
        }

        if (o.dump) {
            if (flightController.isArmed()) {
                shell.println(F("bench: -dump nur im DISARM Modus"));
                return -1;
            }
            if (!recorder::dump(shell, o.lastN)) {
                shell.println(F("bench: Aufzeichnung laeuft noch"));
                return -1;
            }
            return 0;
        }

        if (o.stop) {
            flightController.stopRun();
            shell.println(F("[CLI] Pruefstand beendet"));
            return 0;
        }

        if (o.go) {
            if (!validateRun(o)) return -1;
            if (!o.axisSet) {
                shell.println(F("bench: erst Achse waehlen (-roll oder -pitch)"));
                return -1;
            }
            if (!flightController.isArmed()) {
                shell.println(F("bench: erst 'arm'"));
                return -1;
            }
            if (!confirmStart(g_benchPendingMs, F("Pruefstand"))) return 0;

            shared::BenchCfg cfg;
            cfg.limitDeg  = o.limitDeg;
            cfg.timeoutMs = o.timeoutS * 1000UL;
            flightController.startBench(o.axis, o.throttle, o.maxUs, cfg);

            shell.print(F("[CLI] Pruefstand laeuft: "));
            shell.print(o.axis == shared::AXIS_ROLL ? F("roll") : F("pitch"));
            shell.print(F(", throttle=")); shell.print((int)o.throttle);
            shell.print(F(", max="));      shell.print((int)o.maxUs);
            shell.print(F(", limit="));    shell.print(o.limitDeg, 1);
            shell.println(F(" - 'd' bricht sofort ab"));
            return 0;
        }

        shell.print(F("bench: axis="));
        shell.print(o.axis == shared::AXIS_ROLL ? F("roll") : F("pitch"));
        shell.print(F(" samples=")); shell.print(recorder::count());
        shell.print('/');            shell.println(recorder::capacity());
        return 0;
    }

    // ── Relay-Feedback-Autotune ────────────────────────────────────────

    // Einstellregeln. Ti/Td als Vielfache von Tu, wie ueblich tabelliert;
    // umgerechnet wird auf die Parallelform, die PIDController verwendet:
    //   Kp, Ki = Kp/Ti, Kd = Kp*Td
    struct TuneRule { const char *name; float kp; float ti; float td; };
    const TuneRule TUNE_RULES[] = {
        // Klassisch. Zielt auf ein Amplitudenverhaeltnis von 1:4 je Periode,
        // also ~25 % Ueberschwingen und nur etwa Faktor 2 Verstaerkungsreserve.
        {"zn",     0.60f, 0.500f, 0.125f},
        {"pi",     0.45f, 0.833f, 0.000f},
        {"pessen", 0.70f, 0.400f, 0.150f},
        {"some",   0.33f, 0.500f, 0.333f},
        // Vorgabe. Der Pruefstand bildet nicht alle Totzeiten des Flugs ab
        // (ESC-Ansprechzeit, Propellerhochlauf, Rahmenelastizitaet), deshalb
        // ist hier bewusst viel Reserve eingebaut.
        {"no",     0.20f, 0.500f, 0.333f},
    };

    const TuneRule *findRule(const char *name) {
        for (const TuneRule &r : TUNE_RULES)
            if (strcasecmp(name, r.name) == 0) return &r;
        return nullptr;
    }

    void tuneHelp() {
        shell.println(F("tune - PID-Werte automatisch ermitteln (Relay-Feedback)"));
        shell.println(F("usage: tune [-roll|-pitch] [-h us] [-eps deg] [-throttle us]"));
        shell.println(F("            [-cycles n] [-rule name] [-go|-stop|-show]"));
        shell.println(F("            [-apply] [-ki] [-save]"));
        shell.println(F("  -h us       Relais-Amplitude (Vorgabe 60)"));
        shell.println(F("  -eps deg    Hysterese (Vorgabe 1.0)"));
        shell.println(F("  -cycles n   auszuwertende Perioden (Vorgabe 6)"));
        shell.println(F("  -rule name  zn | pi | pessen | some | no (Vorgabe no)"));
        shell.println(F("  -show       letztes Ergebnis erneut anzeigen"));
        shell.println(F("  -apply      Kp und Kd uebernehmen"));
        shell.println(F("  -ki         zusaetzlich Ki (bewusst getrennt, s.u.)"));
        shell.println(F("  -save       uebernehmen und ins EEPROM (nur disarmt)"));
        shell.println(F("Ein Zweipunktregler treibt die Achse in eine Dauer-"));
        shell.println(F("schwingung. Daraus folgen Tu (Periode) und Ku=4h/(pi*a)."));
        shell.println(F("Hilfe hier mit -help, nicht -h: -h ist die Amplitude."));
        shell.println(F("Ki bleibt opt-in, weil das Integral hier schnell in die"));
        shell.println(F("Begrenzung laeuft und am Pruefstand ohnehin nicht greift."));
        shell.println(F("Das Ergebnis ist ein STARTWERT, kein Endergebnis."));
    }

    // Ku aus Relaisamplitude und gemessener Schwingungsamplitude.
    // Die Hysterese eps geht mit ein - ohne sie waere Ku bei verrauschtem
    // Signal systematisch zu klein.
    float computeKu(const shared::TuneResult &r) {
        float root = r.amp * r.amp - r.eps * r.eps;
        return (root > 0.0f) ? (4.0f * r.h) / (PI * sqrtf(root)) : 0.0f;
    }

    // Rechnet aus Ku/Tu die Koeffizienten und gibt alles als JSON aus.
    void printTuneResult(const shared::TuneResult &r, const TuneRule &rule,
                         bool applied, bool withKi) {
        float ku = computeKu(r);
        float kp = rule.kp * ku;
        float ti = rule.ti * r.tu;
        float td = rule.td * r.tu;
        float ki = (ti > 0.0f) ? kp / ti : 0.0f;
        float kd = kp * td;

        shell.print(F("{\"axis\":\""));
        shell.print(r.axis == shared::AXIS_ROLL ? F("roll") : F("pitch"));
        shell.print(F("\",\"h\":"));    shell.print(r.h, 1);
        shell.print(F(",\"eps\":"));    shell.print(r.eps, 2);
        shell.print(F(",\"a\":"));      shell.print(r.amp, 3);
        shell.print(F(",\"Tu\":"));     shell.print(r.tu, 4);
        shell.print(F(",\"Ku\":"));     shell.print(ku, 3);
        shell.print(F(",\"n\":"));      shell.print(r.n);
        shell.print(F(",\"spread\":")); shell.print(r.spread, 3);
        shell.print(F(",\"rule\":\"")); shell.print(rule.name);
        shell.print(F("\",\"Kp\":"));   shell.print(kp, 4);
        shell.print(F(",\"Ki\":"));     shell.print(ki, 4);
        shell.print(F(",\"Kd\":"));     shell.print(kd, 4);
        shell.print(F(",\"applied\":"));
        if (!applied) shell.print(F("[]"));
        else if (withKi) shell.print(F("[\"Kp\",\"Ki\",\"Kd\"]"));
        else shell.print(F("[\"Kp\",\"Kd\"]"));
        shell.println('}');

        if (r.spread > 0.15f)
            shell.println(F("WARNUNG: Perioden streuen >15% - Ergebnis unsicher"));
    }

    int cmdTune(int argc, char **argv) {
        if (argc >= 2 && optIs(argv[1], "-help")) {
            tuneHelp();
            return 0;
        }

        RunOpts o;
        o.limitDeg = TUNE_LIMIT_DEG;
        o.timeoutS = TUNE_TIMEOUT_S;

        float hAmp = TUNE_H_US, eps = TUNE_EPS_DEG;
        uint8_t cycles = TUNE_CYCLES;
        bool doApply = false, doKi = false, doSave = false, doShow = false;
        const TuneRule *rule = findRule("no");

        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i];
            bool handled;
            if (!parseRunOpt(a, i, argc, argv, o, handled)) return -1;
            if (handled) continue;

            if      (optIs(a, "-apply")) { doApply = true; continue; }
            else if (optIs(a, "-ki"))    { doKi = true; doApply = true; continue; }
            else if (optIs(a, "-save"))  { doSave = true; doApply = true; continue; }
            else if (optIs(a, "-show"))  { doShow = true; continue; }

            const bool isH   = optIs(a, "-h");
            const bool isEps = optIs(a, "-eps");
            const bool isCyc = optIs(a, "-cycles");
            const bool isRul = optIs(a, "-rule");
            if (isH || isEps || isCyc || isRul) {
                if (i + 1 >= argc) {
                    shell.print(F("tune: Wert fehlt nach ")); shell.println(a);
                    return -1;
                }
                const char *v = argv[++i];
                if (isRul) {
                    rule = findRule(v);
                    if (!rule) {
                        shell.print(F("tune: unbekannte Regel ")); shell.println(v);
                        shell.println(F("erlaubt: zn pi pessen some no"));
                        return -1;
                    }
                    continue;
                }
                float f;
                if (!parseFloatDe(v, f)) {
                    shell.print(F("tune: keine gueltige Zahl nach ")); shell.println(a);
                    return -1;
                }
                if      (isH)   hAmp   = f;
                else if (isEps) eps    = f;
                else            cycles = (uint8_t)f;
                continue;
            }

            shell.print(F("tune: unbekannte Option ")); shell.println(a);
            shell.println(F("tune -help fuer Hilfe"));
            return -1;
        }

        // ── Ergebnis anzeigen / uebernehmen ────────────────────────────
        if (doShow || doApply) {
            shared::TuneResult r;
            shared::resultRead(r);
            if (!r.valid) {
                shell.println(F("tune: kein gueltiges Ergebnis - erst 'tune -go'"));
                if (r.abortCode == shared::ABORT_NOSWITCH)
                    shell.println(F("      (letzter Lauf: keine Schwingung, -h erhoehen)"));
                return -1;
            }
            bool applied = false;
            if (doApply) {
                if (doSave && flightController.isArmed()) {
                    // EEPROM.commit() haelt beide Kerne an (rp2040.idleOtherCore),
                    // die Motoren stuenden waehrenddessen auf ihrem letzten Wert.
                    shell.println(F("tune: -save nur im DISARM Modus"));
                    return -1;
                }

                float ku = computeKu(r);
                float kp = rule->kp * ku;
                float ti = rule->ti * r.tu;
                float td = rule->td * r.tu;
                float ki = (ti > 0.0f) ? kp / ti : 0.0f;
                float kd = kp * td;

                // Nicht still auf 255 klemmen lassen: _clampCoeff ist als
                // Tippfehlerschutz gedacht, nicht als Ventil fuer eine
                // Rechnung, die aus dem Ruder gelaufen ist.
                float worst = max(kp, max(kd, doKi ? ki : 0.0f));
                if (worst > PID_COEFF_MAX || kp < 0.0f) {
                    shell.print(F("tune: Wert ausserhalb 0..255 ("));
                    shell.print(worst, 2);
                    shell.println(F(") - nicht uebernommen"));
                    return -1;
                }

                PIDController &p = (r.axis == shared::AXIS_ROLL)
                                 ? flightController.getPidRoll()
                                 : flightController.getPidPitch();
                p.setKp(kp);
                p.setKd(kd);
                if (doKi) p.setKi(ki);
                flightController.publish();
                applied = true;

                if (doSave) {
                    saveAllPid();
                    shell.println(F("[CLI] alle drei Regler ins EEPROM gesichert"));
                }
            }

            printTuneResult(r, *rule, applied, doKi);
            if (applied) {
                shell.println(F("Startwert vom Pruefstand - im freien Flug zuerst"));
                shell.println(F("Kd halbieren und mit Ki=0 beginnen."));
            }
            return 0;
        }

        if (o.stop) {
            flightController.stopRun();
            shell.println(F("[CLI] Autotune abgebrochen"));
            return 0;
        }

        if (o.go) {
            if (!validateRun(o)) return -1;
            if (!o.axisSet) {
                shell.println(F("tune: erst Achse waehlen (-roll oder -pitch)"));
                return -1;
            }
            if (hAmp < 5.0f || hAmp > 400.0f) {
                shell.println(F("tune: h ausserhalb 5..400 us"));
                return -1;
            }
            if (eps <= 0.0f || eps >= 15.0f) {
                shell.println(F("tune: eps ausserhalb 0..15 Grad"));
                return -1;
            }
            if (cycles < 2 || cycles > 16) {
                shell.println(F("tune: cycles ausserhalb 2..16"));
                return -1;
            }
            if (!flightController.isArmed()) {
                shell.println(F("tune: erst 'arm'"));
                return -1;
            }
            if (!confirmStart(g_tunePendingMs, F("Autotune"))) return 0;

            shared::TuneCfg cfg;
            cfg.h         = hAmp;
            cfg.eps       = eps;
            cfg.cycles    = cycles;
            cfg.warmup    = TUNE_WARMUP_CYCLES;
            cfg.limitDeg  = o.limitDeg;
            cfg.timeoutMs = o.timeoutS * 1000UL;
            flightController.startTune(o.axis, o.throttle, o.maxUs, cfg);

            shell.print(F("[CLI] Autotune laeuft: "));
            shell.print(o.axis == shared::AXIS_ROLL ? F("roll") : F("pitch"));
            shell.print(F(", h="));      shell.print(hAmp, 0);
            shell.print(F(", eps="));    shell.print(eps, 1);
            shell.print(F(", cycles=")); shell.println(cycles);
            shell.println(F("[CLI] danach 'tune -show' bzw. 'tune -apply'"));
            return 0;
        }

        shell.println(F("tune: -go zum Starten, -help fuer Optionen"));
        return 0;
    }
}

namespace cli {
    void begin(Stream &stream) {
        cliStream = &stream;

        // Benennung: Aktionen als Verben, Werte als setX/getX (analog
        // PIDController::setKp()/getKp()).
        //
        // WICHTIG: Kein Kommandoname darf mit 'd' beginnen! 'd' wird in
        // update() byte-sofort als Not-Aus abgefangen, bevor die Shell es
        // sieht - ein Befehl "disarm" wuerde daher schon beim ersten Byte
        // ausloesen und "isarm" im Puffer hinterlassen. Deshalb "stop".
        // Muss vor der eingebauten Liste stehen -> siehe cmdHelp()
        shell.addCommand(F("help - Kommandos und Sofort-Tasten"), cmdHelp);

        shell.addCommand(F("arm - ARM (2x innerhalb 3 s bestaetigen)"), cmdArm);
        shell.addCommand(F("stop - DISARM, Motoren sofort stoppen"), cmdStop);
        shell.addCommand(F("recalibrate - Barometer rekalibrieren (nur disarmt)"), cmdRecalibrate);
        shell.addCommand(F("statusLog - Statusausgabe ein/aus"), cmdStatusLog);

        shell.addCommand(F("setHeight cm - Zielhoehe setzen"), cmdSetHeight);
        shell.addCommand(F("getHeight - Zielhoehe ausgeben"), cmdGetHeight);
        shell.addCommand(F("getArmed - Flugzustand ausgeben"), cmdGetArmed);
        shell.addCommand(F("pid - PID anzeigen/setzen, 'pid -h' fuer Optionen"), cmdPid);

        shell.addCommand(F("stats - Lageregelkreis Kern 1 als JSON"), cmdStats);
        shell.addCommand(F("bench - Pruefstand, 'bench -h' fuer Optionen"), cmdBench);
        shell.addCommand(F("tune - Autotune, 'tune -help' fuer Optionen"), cmdTune);

        shell.attach(stream);

        // Lebenszeichen auf dem Shell-Kanal: LOGGER_NOTICE() geht je nach
        // _SERIAL_LOG/_BT_LOG ggf. woandershin, dann waere hier sonst nichts
        // zu sehen und der Kanal wirkte tot.
        shell.println();
        shell.println(F("[CLI] bereit - 'help' listet alle Befehle"));
        shell.println(F("[CLI] sofort ohne Enter: d = DISARM, +/- = Zielhoehe +/-10 cm"));
    }

    bool update() {
        if (!cliStream) return false;

        bool consumed = false;

        // Not-Aus-Tasten vor der Shell abfangen - nur am Zeilenanfang, denn
        // mitten in einer Zeile gehoert '-' zum Argument ("setHeight -10").
        while (atLineStart && cliStream->available()) {
            int c = cliStream->peek();
            if (c == 'd' || c == 'D') {
                cliStream->read();
                // Reihenfolge ist Absicht: erst das Flag, das Kern 1 als
                // allererstes prueft, dann selbst die PWM-Register auf
                // Minimum - dieser Kern wartet nicht auf die Mitarbeit des
                // anderen. disarm() loggt und veroeffentlicht danach.
                shared::g_estop = true;
                flightController.getMotors().stopFast();
                flightController.disarm();
                shell.println(F("[CLI] DISARM - Motoren gestoppt"));
            } else if (c == '+') {
                cliStream->read();
                flightController.adjustTargetHeight(10.0f);
                printTargetHeight();
            } else if (c == '-') {
                cliStream->read();
                flightController.adjustTargetHeight(-10.0f);
                printTargetHeight();
            } else {
                break;
            }
            lastByteMs = millis();
            consumed   = true;
        }

        bool hadInput = cliStream->available() > 0;
        if (hadInput) lastByteMs = millis();

        if (shell.executeIfInput()) {
            atLineStart = true;   // Zeile abgearbeitet -> Not-Aus wieder scharf
            consumed    = true;
        } else if (hadInput) {
            atLineStart = false;  // Teilzeile liegt im Shell-Puffer
            consumed    = true;
        } else if (!atLineStart && (millis() - lastByteMs) > IDLE_RESET_MS) {
            // Eine abgebrochene Eingabe darf den Not-Aus nicht dauerhaft
            // blockieren - Puffer verwerfen und wieder scharf schalten.
            shell.resetBuffer();
            shell.println();
            shell.println(F("[CLI] Eingabe verworfen (Timeout)"));
            atLineStart = true;
        }

        return consumed;
    }
}

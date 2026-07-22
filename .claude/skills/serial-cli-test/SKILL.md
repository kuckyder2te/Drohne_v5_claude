---
name: serial-cli-test
description: Fährt den seriellen CLI-Test der Drohnen-Firmware über USB/COM11 (SimpleSerialShell) — verbindet, ruft `help`, sweept `setHeight`-Werte und liest per `getHeight` das targetHeightCm-Clamping [5..100] cm zurück. Verwenden, wenn der Nutzer sinngemäß bittet, die serielle CLI / das setHeight-Clamping / targetHeightCm über die Schnittstelle zu testen oder zu verifizieren.
---

# Serieller CLI-Test (COM11)

Reproduzierbarer Test der USB-CLI der Firmware. Setzt Zielhöhen über `setHeight`
und liest sie per `getHeight` zurück, um das Clamping zu verifizieren.

## Sicherheit (verbindlich)

- **Niemals armen.** Über die USB-CLI ist Armen ohnehin nicht möglich (Armen läuft
  nur über den BT-Kanal, Taste `a`). Es wird ausschließlich `targetHeightCm` gesetzt
  — solange die Drohne disarmt ist, bewegt sich **kein Motor**. Sende nie `a`/`A`.
- Wenn der Port belegt ist (`Zugriff verweigert` / `PermissionError`): einen
  offenen seriellen Monitor (PlatformIO-Monitor, VS-Code-Serial-Monitor, PuTTY)
  auf COM11 als Ursache nennen und den Nutzer um Freigabe bitten. **Prozesse nur
  nach ausdrücklicher Bestätigung des Nutzers beenden**, nie eigenmächtig.

## Voraussetzungen

- Pico angeschlossen, per `pio device list` als **COM11** sichtbar
  (USB VID:PID `2E8A:...`). Falls anderer Port: `--port COMx`.
- Auf dem Pico läuft die **normale Firmware** (nicht ein `test_*`-Tool) — nur sie
  bietet die CLI. Ggf. vorher `pio run --target upload`.
- COM11 ist frei (kein anderer Monitor offen).

## Wissenswertes zur Schnittstelle

- CLI liegt auf **USB/COM11 @ 115200**. Die `LOG`-Ausgaben der Firmware laufen
  dagegen über **BT (Serial1/COM4)** — auf COM11 erscheint nur die CLI-Antwort.
- Die Shell terminiert Kommandos mit **`\r`** (nicht `\n`); `\n`-Eingaben stauen
  sich im Puffer und verstümmeln das nächste Kommando. Das Skript flusht daher vor
  jedem Kommando mit einem einzelnen `\r`.
- Befehle: `help`, `getHeight`, `setHeight <cm>`. `setHeight` clampt auf
  `[THROTTLE_MIN_CM=5 .. MAX_HEIGHT_CM=100]` (siehe [config.h](../../../include/config.h),
  [cli.cpp](../../../src/comm/cli.cpp), `FlightController::setTargetHeightCm`).

## Ausführung

Skript mit dem PlatformIO-Python (hat pyserial) starten:

```bash
~/.platformio/penv/Scripts/python.exe .claude/skills/serial-cli-test/height_test.py
```

Optional eigene Werte statt des Standard-Sweeps `-10 .. 150`:

```bash
~/.platformio/penv/Scripts/python.exe .claude/skills/serial-cli-test/height_test.py --port COM11 20 55 90 130
```

Unter PowerShell den Python-Pfad quoten:
`& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" ...`

## Danach

Ergebnisse als kurze Tabelle zusammenfassen (Eingabe → resultierendes
`targetHeightCm`) und die Clamp-Grenzen benennen. Erwähnen, dass sich nichts
bewegt hat (disarmt) und ob das beobachtete Verhalten zum Quellcode passt. Falls
zuvor ein Monitor-Prozess beendet wurde, den Nutzer darauf hinweisen, dass er ihn
wieder öffnen kann.

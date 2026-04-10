#include "comm/BluetoothConfig.h"
#include "myLogger.h"
#include "config.h"

void BluetoothConfig::begin()
{
    Serial1.setTX(PIN_BT_TX);
    Serial1.setRX(PIN_BT_RX);
    BT_UART.begin(BT_BAUD);
    BT_UART.println("[BT] Drohne bereit");
    BT_UART.println("[BT] Befehle: P=x.x | I=x.x | D=x.x | S=save | R=reset | ?");
    LOG("[BT] Bluetooth bereit");
}

void BluetoothConfig::update(PIDController &pidHeight,
                             PIDController &pidRoll,
                             PIDController &pidPitch,
                             Settings &settings)
{
    while (BT_UART.available())
    {
        char c = BT_UART.read();
        if (c == '\n' || c == '\r')
        {
            if (_buffer.length() > 0)
            {
                _processCommand(_buffer, pidHeight, pidRoll, pidPitch, settings);
                _buffer = "";
            }
        }
        else
        {
            _buffer += c;
            if (_buffer.length() > 20)
                _buffer = "";
        }
    }
}

void BluetoothConfig::_processCommand(const String &cmd,
                                      PIDController &pidHeight,
                                      PIDController &pidRoll,
                                      PIDController &pidPitch,
                                      Settings &settings)
{

    // ← NEUER BLOCK HIER EINFÜGEN — vor dem "?" Check:
    if (cmd.length() == 1)
    {
        char c = toupper(cmd[0]);
        switch (c)
        {
        case 'A':
            _pendingKey = KeyEvent::KEY_A;
            return;
        case 'S':
            _pendingKey = KeyEvent::KEY_S;
            return;
        case 'R':
            _pendingKey = KeyEvent::KEY_R;
            return;
        case 'H':
            _pendingKey = KeyEvent::KEY_H;
            return;
        case '+':
            _pendingKey = KeyEvent::ARROW_UP;
            return;
        case '-':
            _pendingKey = KeyEvent::ARROW_DOWN;
            return;
        }
    }

    if (cmd == "?")
    {
        BT_UART.println("[BT] === Hoehe ===");
        BT_UART.print("[BT] Kp=");
        BT_UART.println(pidHeight.getKp(), 4);
        BT_UART.print("[BT] Ki=");
        BT_UART.println(pidHeight.getKi(), 4);
        BT_UART.print("[BT] Kd=");
        BT_UART.println(pidHeight.getKd(), 4);
        BT_UART.println("[BT] === Roll ===");
        BT_UART.print("[BT] Kp=");
        BT_UART.println(pidRoll.getKp(), 4);
        BT_UART.println("[BT] === Pitch ===");
        BT_UART.print("[BT] Kp=");
        BT_UART.println(pidPitch.getKp(), 4);
        return;
    }

    // Hoehe: P= I= D=
    // Roll:  RP= RI= RD=
    // Pitch: PP= PI= PD=
    char param0 = toupper(cmd[0]);
    char param1 = toupper(cmd[1]);

    if (param0 == 'R' && param1 == 'P')
    {
        pidRoll.setKp(cmd.substring(3).toFloat());
    }
    else if (param0 == 'R' && param1 == 'I')
    {
        pidRoll.setKi(cmd.substring(3).toFloat());
    }
    else if (param0 == 'R' && param1 == 'D')
    {
        pidRoll.setKd(cmd.substring(3).toFloat());
    }
    else if (param0 == 'P' && param1 == 'P')
    {
        pidPitch.setKp(cmd.substring(3).toFloat());
    }
    else if (param0 == 'P' && param1 == 'I')
    {
        pidPitch.setKi(cmd.substring(3).toFloat());
    }
    else if (param0 == 'P' && param1 == 'D')
    {
        pidPitch.setKd(cmd.substring(3).toFloat());
    }
    else if (cmd.length() >= 3 && cmd[1] == '=')
    {
        // Hoehe P= I= D=
        float value = cmd.substring(2).toFloat();
        switch (param0)
        {
        case 'P':
            pidHeight.setKp(value);
            break;
        case 'I':
            pidHeight.setKi(value);
            break;
        case 'D':
            pidHeight.setKd(value);
            break;
        case 'S':
            settings.save(pidHeight.getKp(),
                          pidHeight.getKi(),
                          pidHeight.getKd());
            break;
        case 'R':
            settings.reset();
            break;
        }
    }
}

void BluetoothConfig::_printValues(PIDController &pid)
{
    BT_UART.print("[BT] Kp=");
    BT_UART.println(pid.getKp(), 4);
    BT_UART.print("[BT] Ki=");
    BT_UART.println(pid.getKi(), 4);
    BT_UART.print("[BT] Kd=");
    BT_UART.println(pid.getKd(), 4);
}
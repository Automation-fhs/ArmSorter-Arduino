#include "can.h"
#include "params.h"

MCP_CAN CAN0(53); // Set CS to pin 10

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; // Array to store serial string
byte data[] = "HelloWd!";

void canSend(unsigned long id, byte lenght, byte *buf)
{
    byte sendStat = CAN0.sendMsgBuf(id, 8, buf);
    uint32_t temp = millis();
    while (sendStat != 0 && millis() - temp >= timeout)
    {
        sendStat = CAN0.sendMsgBuf(id, 8, buf);
        if (millis() - temp >= timeout)
        {
            err = true;
            return;
        }
    }
}

void err_Hndl(byte errCode)
{
    // not yet implemented
}

void cmd_Hndl(byte cmd[5])
{
    byte res[8];
    res[0] = (uint8_t)(canId >> 8);
    res[1] = (uint8_t)(canId);
    res[2] = Error;
    res[3] = CommandCodeFailure;
    res[4] = 0x00;
    res[5] = 0x00;
    res[6] = 0x00;
    res[7] = 0x00;
    if (cmd[0] == 1)
    {
        Serial.println("Arm Opening");
        setpoint = openDeg;
        res[2] = ArmConfirm;
        res[3] = 0x00;
        canSend(centerId, 8, res);
        return;
    }
    else if (cmd[0] == 0)
    {
        Serial.println("Arm Closing");
        setpoint = homeDeg;
        res[2] = ArmConfirm;
        res[3] = 0x00;
        canSend(centerId, 8, res);
        return;
    }
    else
    {
        canSend(centerId, 8, res);
    }
    // not yet implemented
}

void get_Hndl(byte param)
{
    // not yet implemented
}

void set_Hndl(byte value[5])
{
    // not yet implemented
}

void can_init()
{

    while (CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("Error Initializing MCP2515...");
        delay(200);
    }
    Serial.println("MCP2515 Initialized Successfully!");

    CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
}

void can_read()
{
    Serial.println("Data Receiving");
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    uint32_t senderID = (rxBuf[0] << 8) + rxBuf[1];
    if (rxId == canId && senderID == 0x0000)
    {
        byte cmd[5];
        for (int i = 0; i < 5; i++)
        {
            cmd[i] = rxBuf[i + 3];
        }
        Serial.println(rxBuf[2]);
        switch (rxBuf[2])
        {
        case Error:
            err_Hndl(rxBuf[3]);
            break;
        case CenterCommand:
            cmd_Hndl(cmd);
            break;
        case CenterGETparams:
            get_Hndl(rxBuf[3]);
            break;
        case CenterSETparams:
            set_Hndl(cmd);
            break;
        default:
            byte res[8];
            res[0] = (uint8_t)(canId >> 8);
            res[1] = (uint8_t)(canId);
            res[2] = Error;
            res[3] = MsgCodeFailure;
            res[4] = 0x00;
            res[5] = 0x00;
            res[6] = 0x00;
            res[7] = 0x00;
            CAN0.sendMsgBuf(centerId, 8, res);
            break;
        }
    }

    // if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
    //     sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    // else
    //     sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    // Serial.print(msgString);

    // if ((rxId & 0x40000000) == 0x40000000)
    // { // Determine if message is a remote request frame.
    //     sprintf(msgString, " REMOTE REQUEST FRAME");
    //     Serial.print(msgString);
    // }
    // else
    // {
    //     for (byte i = 0; i < len; i++)
    //     {
    //         sprintf(msgString, " 0x%.2X", rxBuf[i]);
    //         Serial.print(msgString);
    //     }
    // }
    // Serial.println();
    return;
}

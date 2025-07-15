#include "Arm-RS485.h"
#include "Arduino.h"
#define Center 90

Arm_RS485::Arm_RS485(uint8_t RX, uint8_t TX, uint8_t RW, int addr, long speed) : _myRS485(RX, TX)
{
    this->_rs485_speed = speed;
    this->_address = addr;
    this->_rw_Pin = RW;
}

void Arm_RS485::begin()
{
    this->_myRS485.begin(this->_rs485_speed);
}

uint8_t Arm_RS485::message_read(int arr[])
{
    if (this->_myRS485.available())
    {
        // ----- Read message -----
        String received = "";
        bool msgEnd = false;
        while (this->_myRS485.available())
        {
            char temp;
            temp = (char)this->_myRS485.read();
            if (msgEnd == true)
            {
                if (temp == 't')
                    break;
                else
                    return 0x51;
            }
            if (temp == '/')
            {
                msgEnd = true;
                received += temp;
            }
            else
                received += temp;
        }
        char reicv_arr[received.length() + 1];
        strcpy(reicv_arr, received.c_str());
        // ------------------------
        // ----- Char to Array -----
        return this->msg_to_array(arr, reicv_arr);
    }
    else
        return 0x00;
}

uint8_t Arm_RS485::message_write(int code, int data)
{
    int res[5] = {this->_address, Center, code, data, 0};
    res[4] = cs_gen(res);
    char message[100];
    array_to_msg(res, message);
    digitalWrite(this->_rw_Pin, HIGH);
    this->_myRS485.write(message);
    digitalWrite(this->_rw_Pin, LOW);
}

uint8_t Arm_RS485::msg_to_array(int arr[5], char msg[])
{
    for (int i = 0; i < 5; i++)
    {
        arr[i] = 0;
    }
    Serial.println();
    int count = 0;
    int i = -1;
    while (msg[++i] != '/')
    {
        if (msg[i] == ':')
            count += 1;
        else
        {
            switch (count)
            {
            case 0:
            {
                arr[0] = arr[0] * 10 + (int)msg[i] - (int)'0';
            }
            break;
            case 1:
            {
                arr[1] = arr[1] * 10 + (int)msg[i] - (int)'0';
            }
            break;
            case 2:
            {
                arr[2] = arr[2] * 10 + (int)msg[i] - (int)'0';
            }
            break;
            case 3:
            {
                arr[3] = arr[3] * 10 + (int)msg[i] - (int)'0';
            }
            break;
            case 4:
            {
                arr[4] = arr[4] * 10 + (int)msg[i] - (int)'0';
            }
            break;
            }
        }
    }
    if (count == 4)
    {
        if (arr[0] == 90)
        {
            if (arr[1] == this->_address)
            {
                if (cs_check(arr) == 0xFF)
                {
                    return 0xFF;
                }
                else
                    return 0x10;
            }
            else
                return 0xF1;
        }
        else
            return 0xF0;
    }
    else
        return 0x50;
}

void Arm_RS485::array_to_msg(int arr[5], char msg[])
{
    arr[4] = this->cs_gen(arr);
    String temp = "";
    for (int i = 0; i <= 4; i++)
    {
        temp += arr[i];
        if (i != 4)
            temp += ":";
    }
    // temp += "/t";
    Serial.println(temp);
    strcpy(msg, temp.c_str());
};

uint8_t Arm_RS485::cs_check(int arr[5])
{
    for (int i = 0; i < 5; i++)
    {
        Serial.print(arr[i]);
        Serial.print(":");
    }
    if (this->cs_gen(arr) == arr[4])
    {
        return 0xFF;
    }
    else
    {
        return 0x10;
    }
}

int Arm_RS485::cs_gen(int arr[5])
{
    int digit1 = (arr[0] % 10 + (arr[0] - arr[0] % 10) / 10) % 10;
    int digit2 = (arr[1] % 9 + (arr[1] - arr[1] % 9) / 9) % 9;
    int digit3 = (arr[2] % 10 + (arr[2] - arr[2] % 10) / 10) % 10;
    int digit4 = (arr[3] % 9 + (arr[3] - arr[3] % 9) / 9) % 9;
    int checksum = ((digit1 * 10 + digit2) + 2 * (digit3 * 10 + digit4)) % 100;
    return checksum;
}
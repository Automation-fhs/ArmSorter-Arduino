#include <SoftwareSerial.h>

class Arm_RS485
{
public:
    Arm_RS485(uint8_t RX, uint8_t TX, uint8_t RW, int addr, long speed);
    void begin();
    uint8_t message_read(int msg[]);
    uint8_t message_write(int code, int data);
    uint8_t msg_to_array(int arr[5], char msg[]);
    void array_to_msg(int arr[5], char msg[]);
    uint8_t cs_check(int arr[5]);
    int cs_gen(int arr[5]);

private:
    int _address;
    long _rs485_speed;
    uint8_t _rw_Pin;
    SoftwareSerial _myRS485;
};
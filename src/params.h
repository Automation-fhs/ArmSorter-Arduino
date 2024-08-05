#define centerId 0x0000

// Message code
#define Error 0x00
#define CenterCommand 0x01
#define ArmConfirm 0x02
#define ArmSensor 0x03
#define CenterGETparams 0x04
#define CenterSETparams 0x05
#define ArmRespond 0x06
#define ArmConfChange 0x07
#define TestCommand 0x08

// Error code
#define MsgCodeFailure 0x00
#define CommandCodeFailure 0x01

// PARAM code
#define isSensor 0x01

#define timeout 1000

extern unsigned long canId;
extern bool err;
extern float setpoint;
extern float openDeg;
extern float homeDeg;
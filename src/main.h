#define TIMER_INTERRUPT_DEBUG 2
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define USE_TIMER_1 true

#if (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) ||                   \
     defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_ETHERNET) ||                        \
     defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT) || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO) ||                            \
     defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) ||    \
     defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
     defined(ARDUINO_AVR_PROTRINKET3FTDI))
#define USE_TIMER_2 true
#else
#define USE_TIMER_3 true
#endif
#include "MotorControl.h"
#include <TimerInterrupt.h>
#include "params.h"
#include "can.h"
#include <Arduino.h>
#include <EEPROM.h>

#define CAN0_INT 2 // Set INT to pin 2

#define TIMER1_INTERVAL_MS 10
#define TIMER1_FREQUENCY (float)(1000.0f / TIMER1_INTERVAL_MS)
#define Motor_PWM 4
#define Motor_Dir 3
#define Control_Pin 23
// #define Test_Mode 25
#define Enc_B 20
#define Enc_A 21
#define Home_Sensor 25
#define Open_Offset_Mode 27
#define Home_Offset_Mode 29
#define OffsetConfirm 31
#define Offset_Enc_A 33
#define Offset_Enc_B 35
#define HomeSpeed 30
#define Enc_Z
#define Home_Offset_Addr 0
#define Open_Offset_Addr 1
#define Motor_Voltage 12
#define Enc_PPP 1000
#define PkgSensor 18
#define CallibHome 350

MotorControl Motor1 = MotorControl(Motor_Voltage, Enc_PPP);

// ----- Encoder -----

String enc_type = "AB";
int cur_P = 0;

// ----- Motor -----
// float PID[3] = {0.6, 0.2, 0.26};
float PID[3] = {27, 8, 4.5};
// float PID[3] = {34.2, 20, 8.55};

float pidFreq = TIMER1_FREQUENCY;
float setpoint = 0;
int minSpeed = 0;
unsigned long canId = 0x0101;

// ----- Control -----
float openDeg;
float homeDeg;
float prev_setpoint;
bool prev_Home;
bool armed = false;
int contrl_signl;
int prev_contrl_signl;
float prev_pos;
bool isHome = false;
bool sensorSgnlSent = false;
bool useSensor = false;
bool errState = false;

bool err = false;
int led = 0;
uint32_t t = millis();
uint32_t sTimer = millis();
uint32_t uSTimer = millis();

volatile uint32_t Timer1Count = 0;
volatile uint32_t TimerCount = 0;

// float PID[3] = {0.6, 0.2, 0.26};
// float PID[3] = {12, 10, 0.75};

// void TimerHandler1()
// {
//   Serial.println(millis() - t);
//   t = millis();
// }
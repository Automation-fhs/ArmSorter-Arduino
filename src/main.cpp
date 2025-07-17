#include "main.h"

char str[50];
uint32_t homeNR;
bool homeNR_Flag;
uint32_t control_timer;
int prev_control;
long prev_pos;
bool home_err = false;
bool isOpened = false;
uint32_t sortTimer = 0;
bool encCheck = false;
long posdata[3][3] = {{0, -1}, {0, -1}, {0, -1}};
float vel_accel[2];
uint32_t lagTime;
uint32_t homeStuck;
long prevHomePulse;
bool callibHome = false;
bool callibOpen = false;
bool enc_err_test;
bool ard_err_test;
bool prepHomeCallib = false;

uint32_t getLagTime(int control_signal);
void velacc(long curPulse);

bool homeSensor()
{
  return digitalRead(Home_Sensor);
}

bool openSensor()
{
  return !digitalRead(Open_Sensor);
}

int checkEncoder(int curPulse)
{

  if (!openSensor() && !homeSensor())
  {
    return 100;
  }

  int errCode = 0;
  if (curPulse < CallibOpen - 30 && !openSensor())
  {
    errCode = 1;
  }
  else if (curPulse > CallibOpen + 30 && openSensor())
  {
    errCode = 2;
  }

  if (curPulse > CallibHome + 30 && !homeSensor())
  {
    errCode += 20;
  }
  else if (curPulse < CallibHome - 30 && homeSensor())
  {
    errCode += 10;
  }

  return errCode;
}

void homeMode()
{
  Serial.println("Finding Home");
  if (homeSensor())
  {
    
    analogWrite(Motor_Dir, 0);
    analogWrite(Motor_PWM, HomeSpeed);
    homeStuck = millis();
    prevHomePulse = Motor1.getCurPulse();
    int modifiedHomeSpeed = HomeSpeed;
    while (homeSensor())
    {
      // Serial.println("Finding Home");
      if (prevHomePulse - Motor1.getCurPulse() > 1)
      {
        homeStuck = millis();
      }
      if (millis() - homeStuck >= 300)
      {
        modifiedHomeSpeed += 10;
        homeStuck = millis();
      }
      analogWrite(Motor_PWM, modifiedHomeSpeed);
    }
    Motor1.setCurPulse(CallibHome);
    // Serial.println("Home found!");
  }
  else
  {
    analogWrite(Motor_Dir, 255);
    analogWrite(Motor_PWM, HomeSpeed);
    while (!homeSensor())
    {
      analogWrite(Motor_PWM, HomeSpeed);
    }
    analogWrite(Motor_PWM, 0);
    analogWrite(Motor_Dir, 0);
    homeStuck = millis();
    prevHomePulse = Motor1.getCurPulse();
    int modifiedHomeSpeed = HomeSpeed;
    while (homeSensor())
    {
      if (prevHomePulse - Motor1.getCurPulse() > 1)
      {
        homeStuck = millis();
      }
      if (millis() - homeStuck >= 300)
      {
        modifiedHomeSpeed += 10;
        homeStuck = millis();
      }
      analogWrite(Motor_PWM, modifiedHomeSpeed);
    }
    Motor1.setCurPulse(CallibHome);
  }
  analogWrite(Motor_PWM, 0);
}

void enc()
{
  if (!enc_err_test)
    Motor1.upd_Pulse(digitalRead(Enc_A), digitalRead(Enc_B));
}

void encoderCheck(int curSignal, int curPulse);

void pidCall()
{
  if (!errState && !ard_err_test)
  {
    // velacc(Motor1.getCurPulse());
    Serial.print(Motor1.getCurPulse());
    Serial.print(" | ");
    Serial.print(homeSensor());
    Serial.print(" | ");
    Serial.println(openSensor());
    // digitalWrite(NL_Pin, HIGH);
    if (NL_Sgnl > 255)
      NL_Sgnl = 0;
    else
      NL_Sgnl += 6;
  }
  analogWrite(NL_Pin, NL_Sgnl);

  if (setpoint == homeDeg && newsetpoint == false)
    newsetpoint = true;
  // Serial.println("PID Calling");
  if (armed && !errState)
  {
    contrl_signl = Motor1.PID_pos_control(setpoint, TIMER1_INTERVAL_MS / 1000.0f);
    float curDeg = Motor1.getCurDeg();

    // Add more resistance to motor when exceeding open/home position
    if (curDeg >= openDeg + 3)
    {
      contrl_signl -= HomeSpeed;
    }
    if (curDeg <= homeDeg - 3)
    {
      contrl_signl += HomeSpeed;
    }

    // Check if arm is in open position but the sensor still triggered
    // if (setpoint == openDeg && newsetpoint == true && abs(Motor1.getCurDeg() - openDeg) <= 2.5 && contrl_signl <= HomeSpeed && !digitalRead(Home_Sensor))
    // {
    //   Serial.println("Fault Open Position");
    //   newsetpoint = false;
    //   analogWrite(Motor_Dir, 255);
    //   analogWrite(Motor_PWM, 0);
    //   contrl_signl = 0;
    //   armed = false;
    //   home_err = true;
    // }

    // Check failed encoder // old method
    if (abs(contrl_signl) <= HomeSpeed + 50 || abs(Motor1.getCurPulse() - prev_pos) > 1)
    {
      control_timer = millis();
      lagTime = getLagTime(abs(contrl_signl));
    }
    else
    {
      uint32_t newLagTime = getLagTime(abs(contrl_signl));
      if (newLagTime < lagTime)
      {
        control_timer = millis();
        lagTime = newLagTime;
      }
    }
    if (millis() - control_timer >= getLagTime(abs(contrl_signl)))
    {
      analogWrite(Motor_PWM, 0);
      errState = true;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Encoder Error!!");
    }

    // Check failed encoder // new method

    // TODO
    // Measure Open Pulse
    // Check Open sensor PIN

    switch (checkEncoder(Motor1.getCurPulse()))
    {
    case 100: // out of sorting range
    {
      analogWrite(Motor_PWM, 0);
      errState = true;
      NL_Sgnl = 0;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Encoder Error!!");
      Serial.println("Error: 100");
    }
    break;
    case 1:
    {
      // haven't open yet (keep open and recallib)
      contrl_signl = 2 * HomeSpeed;
      callibOpen = true;
      Serial.println("Error: 1");
    }
    break;
    case 2:
    {
      analogWrite(Motor_PWM, 0);
      errState = true;
      NL_Sgnl = 0;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Error: 2");
    }
    break;
    case 10: {
      // haven't home yet (keep home and recallib)
      if(setpoint == HomeDegree) {
        contrl_signl = -2 * HomeSpeed;
      callibHome = true;
      Serial.println("Error:10 Close");
      }
      else if(setpoint == OpenDegree) {
        analogWrite(Motor_PWM, 0);
      errState = true;
      NL_Sgnl = 0;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
        Serial.println("Error:10 Open");
      }
    }
    break;
    case 20: {
      analogWrite(Motor_PWM, 0);
      errState = true;
      NL_Sgnl = 0;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Error: 20");
    }
    break;
    case 11:
    {
      // haven't home yet (keep home and recallib)
      contrl_signl = -2 * HomeSpeed;
      callibHome = true;
      Serial.println("Error:11");
    }
    break;
    case 22:
    {
      analogWrite(Motor_PWM, 0);
      errState = true;
      NL_Sgnl = 0;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Error: 22");
    }
    break;
    }

    if (contrl_signl >= 0)
    {
      analogWrite(Motor_PWM, contrl_signl);
      analogWrite(Motor_Dir, 255);
    }

    else
    {
      analogWrite(Motor_PWM, -contrl_signl);
      analogWrite(Motor_Dir, 0);
    }
    prev_pos = Motor1.getCurPulse();
    // Serial.print("Set point:");
    // Serial.print(setpoint);
    // Serial.print(" | Control Signal:");
    // Serial.print(contrl_signl);
    // Serial.print(" | Current Degree");
    // Serial.println(double(Motor1.getCurDeg()));
  }

  if (home_err)
  {
    if (!homeSensor())
    {
      analogWrite(Motor_Dir, 255);
      analogWrite(Motor_PWM, HomeSpeed);
    }
    else
    {
      armed = true;
      home_err = false;
    }
  }

  // prev_contrl_signl = contrl_signl;
  // prev_pos = Motor1.getCurPulse();
}

void FuzzyCall()
{
  if (armed && !errState)
  {
    contrl_signl = Motor1.Fuzzy_pos_control(setpoint, TIMER1_INTERVAL_MS / 1000.0f);

    // contrl_signl = (contrl_signl + 255) / 2;
    // analogWrite(Motor_PWM, contrl_signl);
    float curDeg = Motor1.getCurDeg();
    if (curDeg >= openDeg + 10)
    {
      contrl_signl = -HomeSpeed;
    }
    if (curDeg <= homeDeg - 10)
    {
      contrl_signl = HomeSpeed;
    }
    // if(curDeg <= 7 && contrl_signl <= -HomeSpeed) {
    //   contrl_signl = -HomeSpeed;
    // }

    if (contrl_signl >= 0)
    {
      analogWrite(Motor_PWM, contrl_signl);
      analogWrite(Motor_Dir, 255);
    }

    else
    {
      analogWrite(Motor_PWM, -contrl_signl);
      analogWrite(Motor_Dir, 0);
    }
    // Serial.print("Set point:");
    // Serial.print(setpoint);
    // Serial.print(" | Control Signal:");
    // Serial.print(contrl_signl);
    // Serial.print(" | Current Degree");
    // Serial.println(Motor1.getCurDeg());
  }
}

void sendSensorSignal()
{
  byte res[8];
  res[0] = (uint8_t)(canId >> 8);
  res[1] = (uint8_t)(canId);
  res[2] = ArmSensor;
  res[3] = 0x01;
  res[4] = 0x00;
  res[5] = 0x00;
  res[6] = 0x00;
  res[7] = 0x00;
  canSend(centerId, 8, res);
}

void sendIsUsingSensor(bool useSensor)
{
  byte res[8];
  res[0] = (uint8_t)(canId >> 8);
  res[1] = (uint8_t)(canId);
  res[2] = ArmRespond;
  res[3] = isSensor;
  res[4] = useSensor;
  res[5] = 0x00;
  res[6] = 0x00;
  res[7] = 0x00;
  canSend(centerId, 8, res);
}

void enc_Z()
{
  Serial.print("Z at: ");
  Serial.println(Motor1.getCurPulse());
  // Motor1.setCurPulse(CallibZ);
}

void reCallib()
{
  // if (!digitalRead(Home_Sensor))
  // {
  //   homeNR = millis();
  //   homeNR_Flag = true;
  // }
  // else
  //   homeNR_Flag = false;
  if ((Motor1.getCurPulse() >= CallibHome - 100 && Motor1.getCurPulse() <= CallibHome + 100 || callibHome) && setpoint == HomeDegree)
  {
    Serial.print("Callib home current pulse: ");
    Serial.println(Motor1.getCurPulse());
    Motor1.setCurPulse(CallibHome);
    callibHome = false;
  }
}

void setup()
{
  t = millis();
  Serial.begin(115200);
  errState = false;
  //--------------------- Motor Init ---------------------
  Motor1.set_PID(PID);
  Motor1.setpwm_res(255);

  //--------------------- CAN Init -----------------------
  // can_init();
  // pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input
  // attachInterrupt(digitalPinToInterrupt(CAN0_INT), can_read, LOW);

  //-------------------- Timer Init ----------------------
  ITimer1.init();
  if (ITimer1.attachInterrupt(TIMER1_FREQUENCY, pidCall))
    Serial.println("Starting Timer1 OK!");
  else
    Serial.println("Can't set Timer1");

  //-------------------- Encoder Init --------------------
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);

  pinMode(Home_Sensor, INPUT);
  Motor1.enc_Init(digitalRead(Enc_A), digitalRead(Enc_B));
  attachInterrupt(digitalPinToInterrupt(Enc_A), enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_B), enc, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(Home_Sensor), reCallib, CHANGE);

  //------------------- Control Init ---------------------
  pinMode(PkgSensor, INPUT);
  pinMode(NL_Pin, OUTPUT);
  pinMode(Control_Pin, INPUT);
  analogWrite(Motor_PWM, 0);
  analogWrite(Motor_Dir, 255);
  enc_err_test = false;
  ard_err_test = false;

  NL_Sgnl = 0;
  // pinMode(Test_Mode, INPUT);

  EEPROM.begin();
  EEPROM.write(0, 1);
  EEPROM.write(1, 0);
  EEPROM.write(2, 0);
  EEPROM.write(3, 0);
  EEPROM.write(4, 0);
  EEPROM.write(5, 1);
  EEPROM.write(6, 3);
  EEPROM.write(7, 0);
  EEPROM.write(8, 0);
  EEPROM.write(9, 0);

  // homeDeg = 2 * ((float)EEPROM.read(0) - 0.5) * (EEPROM.read(1) * 10 + EEPROM.read(2) + ((float)EEPROM.read(3)) / 10 + ((float)EEPROM.read(4)) / 100);
  // openDeg = 2 * ((float)EEPROM.read(5) - 0.5) * (EEPROM.read(6) * 10 + EEPROM.read(7) + ((float)EEPROM.read(8)) / 10 + ((float)EEPROM.read(9)) / 100);
  homeDeg = HomeDegree;
  openDeg = OpenDegree;
  // homeDeg = 0;
  // openDeg = 60;
}

void testMode()
{
  uint32_t startTime = millis();
  // int tempTime = millis();
  setpoint = openDeg;
  while (millis() - startTime < 2000)
  {
  } // Arm open 2sec

  setpoint = homeDeg;
  while (millis() - startTime < 4000)
  {
  } // Arm close 2sec
}

// void offset(String loc)
// {
//   delay(200);
//   float _tempDeg;
//   if (loc.compareTo("Home") == 0)
//   {
//     _tempDeg = 2 * ((float)EEPROM.read(0) - 0.5) * (EEPROM.read(1) * 10 + EEPROM.read(2) + ((float)EEPROM.read(3)) / 10 + ((float)EEPROM.read(4)) / 100);
//     Serial.println("Home");
//   }
//   else if (loc.compareTo("Open") == 0)
//   {
//     _tempDeg = 2 * ((float)EEPROM.read(5) - 0.5) * (EEPROM.read(6) * 10 + EEPROM.read(7) + ((float)EEPROM.read(8)) / 10 + ((float)EEPROM.read(9)) / 100);
//     Serial.println("Open");
//   }
//   Serial.println(_tempDeg);
//   while (((loc.compareTo("Home") == 0) || (loc.compareTo("Open") == 0)) && digitalRead(OffsetConfirm))
//   {
//     setpoint = _tempDeg;
//     if (digitalRead(Offset_Enc_A) != _prev_A)
//     {
//       if (_prev_A != _prev_B)
//       {
//         _tempDeg += 0.02;
//       }
//       else
//       {
//         _tempDeg -= 0.02;
//       }
//       _prev_A = digitalRead(Offset_Enc_A);
//     }
//     if (digitalRead(Offset_Enc_B) != _prev_B)
//     {
//       if (_prev_A != _prev_B)
//       {
//         _tempDeg -= 0.1;
//       }
//       else
//       {
//         _tempDeg += 0.1;
//       }
//       _prev_B = digitalRead(Offset_Enc_B);
//     }
//     Serial.println(_tempDeg);
//   }
//   if (loc.compareTo("Home") == 0)
//   {
//     homeDeg = _tempDeg;
//     if (_tempDeg >= 0)
//       EEPROM.write(0, 1);
//     else
//       EEPROM.write(0, 0);
//     EEPROM.write(1, abs((int)_tempDeg / 10));
//     EEPROM.write(2, abs((int)_tempDeg % 10));
//     EEPROM.write(3, abs((int)(_tempDeg * 10) % 10));
//     EEPROM.write(4, abs((int)(_tempDeg * 100) % 10));
//   }
//   else if (loc.compareTo("Open") == 0)
//   {
//     openDeg = _tempDeg;
//     if (_tempDeg >= 0)
//       EEPROM.write(5, 1);
//     else
//       EEPROM.write(5, 0);
//     EEPROM.write(6, abs((int)_tempDeg / 10));
//     EEPROM.write(7, abs((int)_tempDeg % 10));
//     EEPROM.write(8, abs((int)(_tempDeg * 10) % 10));
//     EEPROM.write(9, abs((int)(_tempDeg * 100) % 10));
//   }
//   setpoint = homeDeg;
//   delay(200);
// }

void getMinSpeed()
{
  float curDeg = Motor1.getCurDeg();
  while (Motor1.getCurDeg() == curDeg)
  {
    minSpeed += 1;
    Serial.print("Min speed = ");
    Serial.println(minSpeed);
    digitalWrite(Motor_Dir, 0);
    analogWrite(Motor_PWM, minSpeed);
  }
}

void loop()
{
  if (!isHome)
  {
    homeMode();
    Serial.println("Home found!");
    isHome = true;
    armed = true;
    setpoint = homeDeg;
    prev_Home = homeSensor();
    prev_setpoint = setpoint;
  }
  // if (homeNR_Flag && millis() - homeNR >= 5)
  // {
  //   // if (Motor1.getCurPulse() >= CallibHome - 100 && Motor1.getCurPulse() <= CallibHome + 100)
  //   // {
  //   //   Serial.print("Callib current pulse: ");
  //   //   Serial.println(Motor1.getCurPulse());
  //   //   Motor1.setCurPulse(CallibHome);
  //   // }
  //   Serial.print("Callib current pulse: ");
  //   Serial.println(Motor1.getCurPulse());
  //   Motor1.setCurPulse(CallibHome);
  //   homeNR_Flag = false;
  // }

  if (callibOpen && setpoint == openDeg)
  {
    Serial.print("Callib open current pulse: ");
    Serial.println(Motor1.getCurPulse());
    Motor1.setCurPulse(CallibOpen);
    callibOpen = false;
  }

  if(!homeSensor()){
    prepHomeCallib = true;
  }

  if ((Motor1.getCurPulse() >= CallibHome - 50 && Motor1.getCurPulse() <= callibHome + 50 || callibHome) && setpoint == HomeDegree && prepHomeCallib && homeSensor())
  {
    Serial.print("Callib home current pulse: ");
    Serial.println(Motor1.getCurPulse());
    Motor1.setCurPulse(CallibHome);
    callibHome = false;
    prepHomeCallib = false;
  }

  if (test == false)
  {
    if (digitalRead(Control_Pin))
    {
      sTimer = millis();
      setpoint = homeDeg;
      if (isOpened == true)
      {
        isOpened = false;
        sortTimer = millis();
      }
    }
    if (millis() - sTimer >= 100)
    {
      setpoint = openDeg;
      isOpened = true;
      // Serial.println("Arm Open");
    }
  }
  /*********************SIGNAL CONTROL MODE********************/
  if (Serial.available())
  {
    int signal = Serial.read();
    if (signal == 49)
    {
      setpoint = openDeg;
      test = true;
    }
    else if (signal == 53)
      enc_err_test = true;
    else if (signal == 57)
      ard_err_test = true;
    else
    {
      setpoint = homeDeg;
      test = false;
    }
  }
}

void encoderCheck(int curSignal, int curPulse)
{
  int testDir = -1;
  if (curSignal >= 0)
  {
    testDir = 0;
  }
  else
  {
    testDir = 1;
  }
  uint32_t startTime = millis();

  while (encCheck)
  {
    analogWrite(Motor_Dir, testDir);
    analogWrite(Motor_PWM, HomeSpeed + 10);
    if (abs(Motor1.getCurPulse() - curPulse) >= 3)
    {
      encCheck = false;
    }
    if (millis() - startTime >= 300)
    {
      analogWrite(Motor_PWM, 0);
      errState = true;
      armed = false;
      contrl_signl = 0;
      analogWrite(Motor_PWM, 0);
      Serial.println("Encoder Error!!");
      encCheck = false;
    }
  }
}

uint32_t getLagTime(int control_signal)
{
  if (control_signal <= 60)
    return 500;
  if (control_signal <= 120)
    return 400;
  if (control_signal <= 200)
    return 350;
  else
    return 300;
}

/*********************************************************************************************************
  END FILE

*********************************************************************************************************/
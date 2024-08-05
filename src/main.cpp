#include "main.h"

char str[50];

void enc()
{
  bool cur_Home = digitalRead(Home_Sensor);
  uint32_t motorPos = Motor1.getCurPulse();
  if (prev_Home == 1 && !cur_Home && motorPos >= 300 && motorPos <= 500)
  {
    Motor1.setCurPulse(CallibHome);
  };
  prev_Home = cur_Home;
  Motor1.upd_Pulse(digitalRead(Enc_A), digitalRead(Enc_B));
  // Serial.print(digitalRead(Enc_A));
  // Serial.println(digitalRead(Enc_B));
  // Serial.println(Motor1.getCurDeg());
}

void pidCall()
{
  if (abs(prev_contrl_signl) >= HomeSpeed && abs(Motor1.getCurPulse() - prev_pos) <= 20)
  {
    errState = true;
    armed = false;
  }

  if (armed && !errState)
  {
    contrl_signl = Motor1.PID_pos_control(setpoint, TIMER1_INTERVAL_MS / 1000.0f);

    // contrl_signl = (contrl_signl + 255) / 2;
    // analogWrite(Motor_PWM, contrl_signl);
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
  }
  prev_contrl_signl = contrl_signl;
  prev_pos = Motor1.getCurPulse();
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

void setup()
{
  t = millis();
  Serial.begin(115200);

  //--------------------- Motor Init ---------------------
  Motor1.set_PID(PID);
  Motor1.setpwm_res(255);

  // can_init();

  // pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

  // attachInterrupt(digitalPinToInterrupt(CAN0_INT), can_read, LOW);

  ITimer1.init();

  if (ITimer1.attachInterrupt(TIMER1_FREQUENCY, pidCall))
    Serial.println("Starting Timer1 OK!");
  else
    Serial.println("Can't set Timer1");

  //------------------ Encoder Interrupt ------------------
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);
  Motor1.enc_Init(digitalRead(Enc_A), digitalRead(Enc_B));
  attachInterrupt(digitalPinToInterrupt(Enc_A), enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_B), enc, CHANGE);
  pinMode(Offset_Enc_A, INPUT);
  pinMode(Offset_Enc_B, INPUT);
  //------------------- Control Init ---------------------
  pinMode(PkgSensor, INPUT);
  pinMode(Control_Pin, INPUT);
  pinMode(Open_Offset_Mode, INPUT);
  pinMode(Home_Offset_Mode, INPUT);
  // pinMode(Test_Mode, INPUT);
  EEPROM.begin();

  EEPROM.write(0, 1);
  EEPROM.write(1, 0);
  EEPROM.write(2, 0);
  EEPROM.write(3, 0);
  EEPROM.write(4, 0);
  EEPROM.write(5, 1);
  EEPROM.write(6, 5);
  EEPROM.write(7, 0);
  EEPROM.write(8, 0);
  EEPROM.write(9, 0);

  homeDeg = 2 * ((float)EEPROM.read(0) - 0.5) * (EEPROM.read(1) * 10 + EEPROM.read(2) + ((float)EEPROM.read(3)) / 10 + ((float)EEPROM.read(4)) / 100);
  openDeg = 2 * ((float)EEPROM.read(5) - 0.5) * (EEPROM.read(6) * 10 + EEPROM.read(7) + ((float)EEPROM.read(8)) / 10 + ((float)EEPROM.read(9)) / 100);

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

void offset(String loc)
{
  delay(200);
  float _tempDeg;
  if (loc.compareTo("Home") == 0)
  {
    _tempDeg = 2 * ((float)EEPROM.read(0) - 0.5) * (EEPROM.read(1) * 10 + EEPROM.read(2) + ((float)EEPROM.read(3)) / 10 + ((float)EEPROM.read(4)) / 100);
    Serial.println("Home");
  }
  else if (loc.compareTo("Open") == 0)
  {
    _tempDeg = 2 * ((float)EEPROM.read(5) - 0.5) * (EEPROM.read(6) * 10 + EEPROM.read(7) + ((float)EEPROM.read(8)) / 10 + ((float)EEPROM.read(9)) / 100);
    Serial.println("Open");
  }
  Serial.println(_tempDeg);

  int _prev_A = digitalRead(Offset_Enc_A);
  int _prev_B = digitalRead(Offset_Enc_B);
  while (((loc.compareTo("Home") == 0) || (loc.compareTo("Open") == 0)) && digitalRead(OffsetConfirm))
  {
    setpoint = _tempDeg;
    if (digitalRead(Offset_Enc_A) != _prev_A)
    {
      if (_prev_A != _prev_B)
      {
        _tempDeg += 0.02;
      }
      else
      {
        _tempDeg -= 0.02;
      }
      _prev_A = digitalRead(Offset_Enc_A);
    }
    if (digitalRead(Offset_Enc_B) != _prev_B)
    {
      if (_prev_A != _prev_B)
      {
        _tempDeg -= 0.1;
      }
      else
      {
        _tempDeg += 0.1;
      }
      _prev_B = digitalRead(Offset_Enc_B);
    }
    Serial.println(_tempDeg);
  }
  if (loc.compareTo("Home") == 0)
  {
    homeDeg = _tempDeg;
    if (_tempDeg >= 0)
      EEPROM.write(0, 1);
    else
      EEPROM.write(0, 0);
    EEPROM.write(1, abs((int)_tempDeg / 10));
    EEPROM.write(2, abs((int)_tempDeg % 10));
    EEPROM.write(3, abs((int)(_tempDeg * 10) % 10));
    EEPROM.write(4, abs((int)(_tempDeg * 100) % 10));
  }
  else if (loc.compareTo("Open") == 0)
  {
    openDeg = _tempDeg;
    if (_tempDeg >= 0)
      EEPROM.write(5, 1);
    else
      EEPROM.write(5, 0);
    EEPROM.write(6, abs((int)_tempDeg / 10));
    EEPROM.write(7, abs((int)_tempDeg % 10));
    EEPROM.write(8, abs((int)(_tempDeg * 10) % 10));
    EEPROM.write(9, abs((int)(_tempDeg * 100) % 10));
  }
  setpoint = homeDeg;
  delay(200);
}

void homeMode()
{
  if (digitalRead(Home_Sensor))
  {
    analogWrite(Motor_Dir, 0);
    while (digitalRead(Home_Sensor))
    {
      // Serial.println("Finding Home");
      analogWrite(Motor_PWM, HomeSpeed);
    }
    // Serial.println("Home found!");
  }
  else
  {
    analogWrite(Motor_Dir, 255);
    while (!digitalRead(Home_Sensor))
    {
      analogWrite(Motor_PWM, HomeSpeed);
    }
    analogWrite(Motor_PWM, 0);
    analogWrite(Motor_Dir, 0);
    while (digitalRead(Home_Sensor))
    {
      analogWrite(Motor_PWM, HomeSpeed);
    }
  }
  analogWrite(Motor_PWM, 0);
  Motor1.setCurPulse(CallibHome);
}

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
  if (errState)
  {
    while (!digitalRead(Home_Offset_Mode) || !digitalRead(Open_Offset_Mode) || !digitalRead(OffsetConfirm))
    {
      Serial.println("Error");
    }
    errState = false;
    isHome = false;
  }
  if (!isHome)
  {
    homeMode();
    isHome = true;
    armed = true;
    setpoint = homeDeg;
    prev_Home = digitalRead(Home_Sensor);
    prev_setpoint = setpoint;
  }
  // // ---------- Test Mode ----------
  // if (digitalRead(Test_Mode))
  // {
  //   testMode();
  // }

  // ----------Home Offset Mode ----------
  if (!digitalRead(Home_Offset_Mode))
  {
    Serial.println("Home_Offset");
    offset("Home");
  }

  // ----------Open Offset Mode ----------
  if (!digitalRead(Open_Offset_Mode))
  {
    Serial.println("Open_Offset");
    offset("Open");
  }

  // if (digitalRead(Control_Pin))
  // {

  //   t = millis();
  //   Motor1.resetIntegral();
  //   while (millis() - t < 1500)
  //   {
  //     setpoint = openDeg;
  //   }
  //   setpoint = homeDeg;
  // }

  /************************CAN CONTROL MODE********************/
  /*
  if (digitalRead(PkgSensor))
  {
    if (useSensor == false)
    {
      useSensor = true;
      Serial.println("Using sensor from now");
      sendIsUsingSensor(useSensor);
      // Using sensor
    }
    sensorSgnlSent = false;
    sTimer = millis();
    uSTimer = millis();
  }
  else
  {
  }

  if (millis() - sTimer >= 100 & !sensorSgnlSent)
  {
    Serial.println("Sensor signal debounced!");
    sensorSgnlSent = true;
    sendSensorSignal();
  }

  if (millis() - uSTimer >= 5000 && useSensor)
  {
    useSensor = false;
    Serial.println("Stop using sensor");
    sendIsUsingSensor(useSensor);
  }
  */
  /**********************CAN CONTROL MODE**********************/

  /*********************SIGNAL CONTROL MODE********************/
  // if (prev_Home == 1 && digitalRead(Home_Sensor) == 0 && Motor1.getCurPulse() >= 250 && Motor1.getCurPulse() <= 450)
  // {
  //   Motor1.setCurPulse(CallibHome);
  // }
  // prev_Home = digitalRead(Home_Sensor);
  if (digitalRead(Control_Pin))
  {
    sTimer = millis();
    setpoint = homeDeg;
  }
  if (millis() - sTimer >= 100)
  {
    setpoint = openDeg;
    // Serial.println("Arm Open");
  }
  // Serial.println(Motor1.getCurDeg());
  /*********************SIGNAL CONTROL MODE********************/
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
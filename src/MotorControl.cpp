#include "MotorControl.h"
#define PI 3.1415926536
int enc_dir = 1;

MotorControl::MotorControl(int rated_V, int enc_pulse_per_phase, String enc_type)
{
    this->_rated_V = rated_V;
    this->enc_pulse_per_phase = enc_pulse_per_phase;
    this->_prev_err = 0;
    this->_integral = 0;
    this->_enc_type = enc_type;
    this->_prev_setpoint = 0;
    this->_new_setpoint = false;
    // for (int i = 0; i < sizeof(this->_enc_type); i++)
    // {
    //     this->_enc_type[i] = enc_type[i];
    // }
}

void MotorControl::Fuzzy_init(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7])
{
    myFuzzy.Init(posCP, posErr, veloCP, veloErr, FuzzyRules);
}

void MotorControl::set_PID(float PID[3])
{
    this->_PID[0] = PID[0];
    this->_PID[1] = PID[1];
    this->_PID[2] = PID[2];
}

void MotorControl::setCurPulse(uint32_t cur_Plse)
{
    this->_cur_pulse = cur_Plse;
}

void MotorControl::setpwm_res(int pwm_res)
{
    this->_pwm_res = pwm_res;
}

void MotorControl::enc_Init(int cur_A, int cur_B)
{
    this->_prev_A = cur_A;
    this->_prev_B = cur_B;
}

void MotorControl::upd_Pulse(int cur_A, int cur_B)
{
    if (cur_A == _prev_A)
    {
        if (cur_A != cur_B)
        {
            _cur_pulse += enc_dir;
        }
        else
        {
            _cur_pulse -= enc_dir;
        }
    }
    else
    {
        if (cur_A != cur_B)
        {
            _cur_pulse -= enc_dir;
        }
        else
        {
            _cur_pulse += enc_dir;
        }
    }
    _prev_A = cur_A;
    _prev_B = cur_B;
}

uint32_t MotorControl::getCurPulse()
{
    return this->_cur_pulse;
}

float MotorControl::getCurDeg()
{
    if (this->_enc_type.compareTo("AB") == 0)
    {
        int absPulse = (this->_cur_pulse + 2 * this->enc_pulse_per_phase) % (4 * this->enc_pulse_per_phase);
        return absPulse * (1 - 2 * 1000 / (float)abs(absPulse)) * 360 / (4 * 1000);
    }
    else
        return 0;
}

float MotorControl::getCurRad()
{
    return getCurDeg() * 3.1415 / 180;
}

void MotorControl::resetIntegral()
{
    this->_integral = 0;
}

int MotorControl::PID_pos_control(float setpoint, float timespan, String unit)
{
    // uint32_t timer = millis();
    float _setpoint;
    bool resetVelo = false;

    // -----Check if new setpoint-----
    if (setpoint != this->_prev_setpoint)
    {
        this->resetIntegral();
        this->_prev_setpoint = setpoint;
        this->_new_setpoint = true;
        resetVelo = true;
    }

    // -----Check if Unit is Degree or Radian-----
    if (unit.compareTo("Deg") == 0)
        _setpoint = setpoint * PI / 180;
    else
        _setpoint = setpoint;
    float err = _setpoint - getCurRad();

    // -----Terminate Integral if error is small -----
    if (abs(err) >= 0.03)
        _integralTimer = millis();
    if (millis() - _integralTimer >= 1000)
        this->resetIntegral();

    // -----Terminate Integral ONCE to prevent overshoot-----
    if (abs(err) <= 0.05 && this->_new_setpoint == true)
    {
        this->resetIntegral();
        this->_new_setpoint = false;
        // Serial.println("Reset Integral!");
    }
    


    _integral += err * timespan;
    float Pout = _PID[0] * err;
    float Iout = _PID[1] * _integral;
    float Dout;
    if (resetVelo == false)
    {
        Dout = _PID[2] * (err - this->_prev_err) / timespan;
    }
    else
    {
        Dout = 0;
        resetVelo = false;
    }

    Serial.print(this->getCurDeg());
    Serial.print(" ");
    //Serial.print("P:");
    Serial.print(Pout);
    Serial.print(" ");
    //Serial.print("I:");
    Serial.print(Iout);
    Serial.print(" ");
    //Serial.print("D:");
    Serial.println(Dout);

    this->_prev_err = err;

    if(abs(err) <= 0.02 && abs(Dout) <= 2.5) return 0;
    // Serial.println(millis() - timer);
    if (Pout + Iout + Dout > this->_rated_V)
        return this->_pwm_res;
    else if (Pout + Iout + Dout < -this->_rated_V)
        return -this->_pwm_res;
    else
        return (int)((Pout + Iout + Dout) * this->_pwm_res / this->_rated_V);

    // Serial.println("Test");
}

int MotorControl::Fuzzy_pos_control(float setpoint, float timespan, String unit = "Deg")
{
    float _setpoint;
    // -----Check if Unit is Degree or Radian-----
    if (unit.compareTo("Deg") == 0)
        _setpoint = setpoint;
    else
        _setpoint = setpoint * 180 / PI;
    float pos = getCurDeg() - setpoint;
    float velo = (pos - this->_prev_err) / timespan;
    this->_prev_err = pos;
    Serial.print(getCurDeg());
    Serial.print(" ");
    Serial.print(velo);
    Serial.print(" ");
    Serial.println(myFuzzy.Result(pos, velo));
    return myFuzzy.Result(pos, velo);
}
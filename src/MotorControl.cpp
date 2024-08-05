#include "MotorControl.h"
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
    uint32_t timer = millis();
    float _setpoint;
    if (setpoint != this->_prev_setpoint)
    {
        this->resetIntegral();
        //Serial.println("Reset Integral!");
        this->_prev_setpoint = setpoint;
        this->_new_setpoint = true;
    }
    if (unit.compareTo("Deg") == 0)
        _setpoint = setpoint * 3.1415 / 180;
    else
        _setpoint = setpoint;
    float err = _setpoint - getCurRad();

    if (abs(err) >= 0.003)
        _integralTimer = millis();
    if (millis() - _integralTimer >= 1000)
        this->resetIntegral();
    if (abs(err) <= 0.01 && this->_new_setpoint == true)
    {
        this->resetIntegral();
        this->_new_setpoint = false;
        //Serial.println("Reset Integral!");
    }

    _integral += err * timespan;
    float Pout = _PID[0] * err;
    float Iout = _PID[1] * _integral;
    float Dout = _PID[2] * (err - this->_prev_err) / timespan;
    // Serial.print(setpoint);
    // Serial.print(" ");
    // // Serial.print("P:");
    // Serial.print(Pout);
    // // Serial.print("I:");
    // Serial.print(" ");
    // Serial.print(Iout);
    // Serial.print(" ");
    // // Serial.print("D:");
    // Serial.println(Dout);

    this->_prev_err = err;
    // Serial.println(millis() - timer);
    if (Pout + Iout + Dout > this->_rated_V)
        return this->_pwm_res;
    else if (Pout + Iout + Dout < -this->_rated_V)
        return -this->_pwm_res;
    else
        return (int)((Pout + Iout + Dout) * this->_pwm_res / this->_rated_V);

    // Serial.println("Test");
}

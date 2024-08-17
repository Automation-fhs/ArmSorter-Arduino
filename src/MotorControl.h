#include <string.h>
#include <Arduino.h>
#include <Fuzzy.h>

class MotorControl
{
public:
    // -----Motor-----
    MotorControl(int rated_V, int enc_pulse_per_phase, String enc_type = "AB");
    void setpwm_res(int pwm_res);
    void Fuzzy_init(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7]);
    // -----Encoder-----
    void enc_Init(int cur_A, int cur_B);
    void upd_Pulse(int cur_A, int cur_B);
    void setCurPulse(uint32_t cur_Plse);
    uint32_t getCurPulse();
    float getCurDeg();
    float getCurRad();
    void resetIntegral();
    // -----PID-----
    void set_PID(float PID[3]);
    int PID_pos_control(float setpoint, float timespan, String unit = "Deg");
    // -----Fuzzy-----
    Fuzzy myFuzzy = Fuzzy();
    int Fuzzy_pos_control(float setpoint, float timespan, String unit = "Deg");

private:
    // ----- Motor -----
    int _rated_V;
    int _pwm_res;

    // ----- Encoder -----
    int enc_pulse_per_phase;
    uint32_t _cur_pulse = 0;
    int _total_pulse;
    int _pulse_per_chnl;
    int _prev_A;
    int _prev_B;
    uint32_t _integralTimer = 0;

    // ----- PID -----
    float _PID[3];
    float _prev_err;
    float _integral;
    String _enc_type;
    float _prev_setpoint;
    bool _new_setpoint;

    // ------ Fuzzy ------
};
#include "CytronMotorDriver.h"

/********************** Functions **********************/
double MapCommand(double x, double in_max, double out_max)
{
  if (x > in_max)
    x = in_max; 
  if (x < -in_max)
    x = -in_max;
  return (x/abs(in_max))*abs(out_max);
}


// MOTOR CLASS
class Motor
{
  public:
    Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold, unsigned int time_period_com, bool switch_sign);
    void setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
        unsigned int max_time_calib, float max_closedloop_vel, float kp, float kd, float ki);
    
    void UpdateLastComm();
    void CheckForComm();
    
    void motor_loop();
    //Goes from -1.0 to 1 if in opened-loop, else its in rad/s
    double vel_setpoint = 0.0;
    double actual_vel = 0.0;

    void StartCalib();
    bool start_calib = false;
    
    bool closed_loop_ctrl = false;
    
  private:
    void SendCmd();
    void DoCalib();
  
    int     _pin_pwm;
    int     _pin_dir;

    double _max_speed;
    double _min_speed_threshold;
      
    CytronMD _motor;

    unsigned long _time_last_com = 0.0; //comm watchdog
    unsigned int _time_period_com;

    // for calibration
    Encoder* _encoder;
    Limitswitch* _switch;

    int     _sign = 1;
    bool    _calibration_setup = false;
    bool    _returning_home_cal = false;
    long    _time_home = 0;
    int     _calib_counter = 0;
    
    int     _calib_dir;
    double  _calib_speed;
    double  _limit_switch_pos; //in radians
    long    _calib_start_time;
    unsigned int     _in_calib_max_time;

    // for closed loop
    float _max_closedloop_vel;
    float _kp, _kd, _ki;
    float _ie, _previous_e;
    unsigned long _previous_t;
    
};


Motor::Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold, unsigned int time_period_com=1000, bool switch_sign=false)
  : _motor(PWM_DIR, pin_pwm, pin_dir)
{   
  _pin_pwm = pin_pwm;
  _pin_dir = pin_dir;

  _max_speed = max_speed;
  _min_speed_threshold = min_speed_threshold;

  _time_period_com = time_period_com;
  
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_dir, OUTPUT);

  if (switch_sign)
    _sign = -1;
}

void Motor::setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
              unsigned int max_time_calib, float max_closedloop_vel, float kp=1., float kd=0., float ki=0.)
{
  // get object pointer
  _encoder = enc;
  _switch = swtch;

  _calib_dir = (calib_dir > 0) - (calib_dir < 0);
  _calib_speed = calib_speed;
  _limit_switch_pos = limit_switch_pos; 

  // Calibration is setup
  _calibration_setup = true;
  _in_calib_max_time = max_time_calib;

  _max_closedloop_vel = max_closedloop_vel;

  _kp = kp;
  _kd = kd;
  _ki = ki;
}

void Motor::UpdateLastComm()
{ 
  _time_last_com = millis();
}

void Motor::CheckForComm()
{
  if (millis() - _time_last_com > _time_period_com)
    vel_setpoint = 0.0;
}

void Motor::SendCmd()
{
  double pwm;
  
  // if in closed loop
  if (_calibration_setup && closed_loop_ctrl)
  {
    unsigned long t = millis();
    // Make sure the main loop is updating actual_vel; 
    float error = vel_setpoint - actual_vel;  
    float cmd = _kp*error;

    if (_kd != 0)
      cmd += _kd*(error-_previous_e)/(t-_previous_t);
    if (_ki != 0)
    {
      _ie += error*(t-_previous_t);
      cmd += _ki*_ie;
    }
    
    _previous_t = t;
    _previous_e = error;

    if (abs(cmd) < 0.005) 
      cmd = 0;
      
    // For now max speed is 10 rad/s
    pwm = MapCommand(vel_setpoint, _max_closedloop_vel, 255.0);
  }
  
  // If in opened loop:
  else
  {
    //Keep only the sign
    if (abs(vel_setpoint) > _max_speed)
      vel_setpoint = abs(vel_setpoint)/vel_setpoint; 
  
    //Scale the setpoint in -255 to 255
    pwm = MapCommand(vel_setpoint, _max_speed, 255.0); 
  
    //If cmd is too small just send 0
    if (abs(pwm) < _min_speed_threshold) 
      pwm = 0; 
  }
  
  _motor.setSpeed(pwm*_sign);
}

void Motor::StartCalib()
{
  start_calib = true;
  _calib_start_time = millis();
}

void Motor::DoCalib()
{  
  // do calib if conditions are met
  if (_calibration_setup && start_calib)
  { 
    if (_switch->get())
    {
      //Debouncing
      _calib_counter += 1; 
      if (_calib_counter > 3)
      {
        _encoder->set_zero(_limit_switch_pos);
        vel_setpoint = 0.0;

        //Wait a tiny bit so that it doesnt require huge acceleration
        if (millis() - _time_home > 1000)
        {        
          _returning_home_cal = true;
          _calib_counter = 0;
        }
      }
      else
        _time_home = millis();
    }
    else
    {
      // if calib is too long stop it
      if (millis() - _calib_start_time > _in_calib_max_time)
        start_calib = false;
      else
      {
        vel_setpoint = _calib_dir*_calib_speed;
        closed_loop_ctrl = false;
      }
      _time_home = millis();
    }

    // returning to 0 pos
    if (_returning_home_cal)
    {
      if ((_encoder->get() <= 0.005 && _limit_switch_pos >= 0.) || (_encoder->get() >= -0.005 && _limit_switch_pos < 0.))
      {
        vel_setpoint = 0.0;
        start_calib = false;
        _returning_home_cal = false;
      }
      else
      {
        vel_setpoint = _calib_dir*_calib_speed*-1.;
        closed_loop_ctrl = false;
      }
    }
  }
}

void Motor::motor_loop()
{ 
  
  DoCalib();
  CheckForComm();
  SendCmd();
}

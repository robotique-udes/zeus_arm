#include "CytronMotorDriver.h"

/********************** Functions **********************/
double MapCommand(double x, double in_max, double out_max)
{
  if (x > in_max)
    x = in_max;    
  return (x/abs(in_max))*abs(out_max);
}


// MOTOR CLASS
class Motor
{
  public:
    Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold, unsigned int time_period_com);
    void setup_calib(Encoder* enc, bool(*read_switch_func)(), double calib_speed, int calib_dir, double limit_switch_pos);
    
    void UpdateLastComm();
    void CheckForComm();
    void SendCmd();
    void motor_loop();
    double vel_setpoint;

    bool start_calib;
    
  private:
    int     _pin_pwm;
    int     _pin_dir;

    double _max_speed;
    double _min_speed_threshold;
      
    CytronMD _motor;

    unsigned long _time_last_com; //comm watchdog
    unsigned int _time_period_com;

    // for calibration
    Encoder* _encoder;
    bool(*_read_switch_func)();
    
    bool    _calibration_setup;
    int     _calib_dir;
    double  _calib_speed;
    double  _limit_switch_pos; //in radians
    
};


Motor::Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold, unsigned int time_period_com = 1000)
  : _motor(PWM_DIR, pin_pwm, pin_dir)
{   
  _pin_pwm = pin_pwm;
  _pin_dir = pin_dir;
  _calibration_setup = false;
  start_calib = false;

  _max_speed = max_speed;
  _min_speed_threshold = min_speed_threshold;

  _time_last_com = 0.0;
  _time_period_com = time_period_com;
  
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_dir, OUTPUT);

  vel_setpoint = 0.0; //Goes from -1.0 to 1

}

void Motor::setup_calib(Encoder* enc, bool(*read_switch_func)(), double calib_speed, int calib_dir, double limit_switch_pos)
{
  // get function pointer
  _encoder = enc;
  _read_switch_func = read_switch_func;

  _calib_dir = (calib_dir > 0) - (calib_dir < 0);
  _calib_speed = calib_speed;
  _limit_switch_pos = limit_switch_pos; 

  // Calibration is setup
  _calibration_setup = true;
}


void Motor::UpdateLastComm()
{ 
  _time_last_com = millis();
}

void Motor::CheckForComm()
{
  if (millis() - _time_last_com > _time_period_com)
    vel_setpoint = 0.0;
    //Serial.println("No comm");
}

void Motor::SendCmd()
{
  //Keep only the sign
  if (abs(vel_setpoint) > _max_speed)
    vel_setpoint = abs(vel_setpoint)/vel_setpoint; 

  //Scale the setpoint in -255 to 255
  double pwm = MapCommand(vel_setpoint, _max_speed, 255.0); 

  //If cmd is too small just send 0
  if (abs(pwm) < _min_speed_threshold) 
    pwm = 0; 

  //Serial.println(pwm);
  _motor.setSpeed(pwm);
}


void Motor::motor_loop()
{ 
  // do calib if conditions are met
  if (_calibration_setup && start_calib)
  {
    if (_read_switch_func())
    {
      _encoder->set_zero(_limit_switch_pos);
      vel_setpoint = 0.0;
      start_calib = false;
    }
    else
    {
      vel_setpoint = _calib_dir*_calib_speed;
    }
  }

  CheckForComm();
  SendCmd();
}

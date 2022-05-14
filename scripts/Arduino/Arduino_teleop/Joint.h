#include <PID_v1.h>
#include "Motor.h"

#define MAX_LIM 2

// JOINT CLASS
class Joint
{
  public:
    Joint(Motor* motor, double max_speed, double min_speed_threshold, unsigned int time_period_com);
    void setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
        unsigned int max_time_calib, float kp, float kd, float ki);

    //Ax limit
    void set_ax_limit(Limitswitch* swtch, int dir);

    // Comm
    void UpdateLastComm();
    void CheckForComm();

    // Loop
    void joint_loop();
    //Goes from -1.0 to 1 if in opened-loop, else its in rad/s
    
    double vel_setpoint = 0.0;
    double actual_vel = 0.0;

    void StartCalib();
    bool start_calib = false;
    
    bool closed_loop_ctrl = false;
    
    
  private:
    void CtrlCmd();
    void DoCalib();
    void AxLimits();
    void SendCmd();

    double _ctrl_cmd;
  
    int     _pin_pwm;
    int     _pin_dir;

    double _max_speed;
    double _min_speed_threshold;
      
    Motor* _motor;

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
    float _kp, _kd, _ki;
    double _pid_output;
    PID _pid;

    // Ax limit
    struct ax_limit
    {
      Limitswitch* swtch;
      int dir;
    };
    ax_limit _limits[MAX_LIM];
    int _n_lim = 0;
    
};


Joint::Joint(Motor* motor, double max_speed, double min_speed_threshold, unsigned int time_period_com=1000)
  : _motor(motor), _pid(&actual_vel, &_pid_output, &vel_setpoint, _kp, _ki, _kd, DIRECT)
{   
  _max_speed = max_speed;
  _min_speed_threshold = min_speed_threshold;

  _time_period_com = time_period_com;
}

void Joint::setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
              unsigned int max_time_calib, float kp=1., float kd=0., float ki=0.)
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

  _kp = kp;
  _kd = kd;
  _ki = ki;
  
  //turn the PID on
  _pid.SetMode(AUTOMATIC);
}

void Joint::set_ax_limit(Limitswitch* swtch, int dir)
{
  if (_n_lim < MAX_LIM)
  {
    _limits[_n_lim] = {swtch, dir};
    _n_lim += 1;
  }
}

void Joint::AxLimits()
{
  for (int i = 0; i<_n_lim; i++)
  {
    if (_limits[i].swtch->get())
    {
      if (_ctrl_cmd*_limits[i].dir > 0.0) //same sign
        _ctrl_cmd = 0.0;
    }
  }
}

void Joint::UpdateLastComm()
{ 
  _time_last_com = millis();
}

void Joint::CheckForComm()
{
  if (millis() - _time_last_com > _time_period_com)
    _ctrl_cmd = 0.0;
}

void Joint::CtrlCmd()
{
  double cmd = vel_setpoint;
  // If in closed loop
  if (_calibration_setup && closed_loop_ctrl)
  {
    double gap = abs(vel_setpoint-actual_vel); //distance away from setpoint
    if (gap < 0.3) //we're close to setpoint, use conservative tuning parameters
       _pid.SetTunings(_kp, _ki, _kd);
    else //we're far from setpoint, use aggressive tuning parameters
       _pid.SetTunings(_kp*4, _ki*4, _kd*4);

    _pid.Compute();
    cmd = _pid_output + actual_vel;
  }
  
  // If in opened loop:
  else
  {
    //Scale the setpoint from -1 to 1
    cmd = MapCommand(cmd, _max_speed, 1.0);  
  }

  //If cmd is too small just send 0
  if (abs(cmd) < _min_speed_threshold) 
    cmd = 0;

  _ctrl_cmd = cmd;
}

void Joint::StartCalib()
{
  start_calib = true;
  _calib_start_time = millis();
}

void Joint::DoCalib()
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
        _ctrl_cmd = 0.0;

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
        _ctrl_cmd = _calib_dir*_calib_speed;
        closed_loop_ctrl = false;
      }
      _time_home = millis();
    }

    // returning to 0 pos
    if (_returning_home_cal)
    {
      if ((_encoder->get() <= 0.005 && _limit_switch_pos >= 0.) || (_encoder->get() >= -0.005 && _limit_switch_pos < 0.))
      {
        _ctrl_cmd = 0.0;
        start_calib = false;
        _returning_home_cal = false;
      }
      else
      {
        _ctrl_cmd = _calib_dir*_calib_speed*-1.;
        closed_loop_ctrl = false;
      }
    }
  }
}
  

void Joint::joint_loop()
{ 
  CtrlCmd();
  
  AxLimits();
  DoCalib();
  CheckForComm();

  _motor->set_speed(_ctrl_cmd);
}

#include "Motor.h"

#define MAX_LIM 2

// JOINT CLASS
class Joint
{
  public:
    Joint(Motor* motor, double max_speed, double min_speed_threshold, unsigned int time_period_com);
    void setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
        unsigned int max_time_calib, float kp, float kd, float ki, unsigned int closed_loop_period, float max_ie);

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

    double debug=1;
    
    
  private:
    void CtrlCmd();
    void DoCalib();
    void AxLimits();
    void SendCmd();

    double _cmd, _ctrl_cmd;
  
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

    // for closed loop PID
    float _kp, _kd, _ki;
    double _previous_e, _ie;
    float _max_ie;
    unsigned int _closed_loop_period;
    unsigned long _previous_t;

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
  : _motor(motor)
{   
  _max_speed = max_speed;
  _min_speed_threshold = min_speed_threshold;

  _time_period_com = time_period_com;
}

void Joint::setup_calib(Encoder* enc, Limitswitch* swtch, double calib_speed, int calib_dir, double limit_switch_pos, 
              unsigned int max_time_calib, float kp=1., float kd=0., float ki=0., unsigned int closed_loop_period=50, float max_ie=100)
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



  // PID
  _max_ie = max_ie;
  _closed_loop_period = closed_loop_period;
 
  _kp = kp;
  _kd = kd;
  _ki = ki;
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
      if (_cmd*_limits[i].dir > 0.0) //same sign
        _cmd = 0.0;
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
    _cmd = 0.0;
}

void Joint::CtrlCmd()
{ 
  // If in closed loop
  if (_calibration_setup && closed_loop_ctrl)
  {
    unsigned long t = millis();
    if ((t-_previous_t)>_closed_loop_period)
    {
      /*
      if (abs(vel_setpoint) < _min_speed_threshold) 
        _ctrl_cmd = 0;
      else
      {*/
        // Make sure the main loop is updating actual_vel; 
        double error = vel_setpoint - actual_vel; 
        
        // KP
        _ctrl_cmd = _kp*error;
        
        // KD
        if (_kd != 0)
        {
          double d_e = ((error-_previous_e)*1000)/(t-_previous_t);
          _ctrl_cmd += _kd*d_e;
        }

        // KI
        if (_ki != 0)
        {
          _ie += error*(t-_previous_t);
          if (abs(_ie) > _max_ie && _ie != 0)
            _ie = (abs(_ie)/_ie)*_max_ie; //keep only the sign of _ie * _max_ie
          _ctrl_cmd += _ki*_ie;
        }
        
        _previous_e = error;
        _ctrl_cmd = _ctrl_cmd + actual_vel;
      //}
      _previous_t = t;
    }
    _cmd = _ctrl_cmd;
  }
  
  // If in opened loop:
  else
  {
    //Scale the setpoint from -1 to 1
    _cmd = MapCommand(vel_setpoint, 1.0, _max_speed);

    //If cmd is too small just send 0
    if (abs(_cmd) < _min_speed_threshold) 
      _cmd = 0;
  }

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
        _cmd = 0.0;

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
        _cmd = _calib_dir*_calib_speed;
        closed_loop_ctrl = false;
      }
      _time_home = millis();
    }

    // returning to 0 pos
    if (_returning_home_cal)
    {
      if ((_encoder->get() <= 0.005 && _limit_switch_pos >= 0.) || (_encoder->get() >= -0.005 && _limit_switch_pos < 0.))
      {
        _cmd = 0.0;
        start_calib = false;
        _returning_home_cal = false;
      }
      else
      {
        _cmd = _calib_dir*_calib_speed*-1.;
        closed_loop_ctrl = false;
      }
    }
  }
}
  

void Joint::joint_loop()
{ 
  CtrlCmd();

  /*
  Serial.print(millis());
  Serial.print(" | ");
  Serial.println(_cmd);
  */
  
  AxLimits();
  DoCalib();
  CheckForComm();

  //_motor->set_speed(_cmd);
  
  debug = _cmd;
}

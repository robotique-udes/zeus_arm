#ifndef _SINGLEJOINTS
#define _SINGLEJOINTS

#include "Motor.h"
#include "Joint.h"
#include "Encoder.h"

#include <PID_v1.h>

// Max number of limitswitch
#define MAX_LIM 2

// JOINT CLASS
class SingleJoint : public Joint
{
  public:
    SingleJoint(Motor* motor, Encoder* enc, double maxSpeed, double maxNegSpeed, 
        unsigned int timePeriodCom, double kp, double ki, double kd);
    
    void setGains(double kp, double ki, double kd);
  
    double actualVel = 0;
    bool reachedMaxSpeed = false;

    double getMaxSpeed() { return _maxSpeed; }
    double getMaxSpeedNeg() { return _maxNegSpeed; }

    void CalculateActVel();
    virtual void JointLoop();


    // for closed loop PID
    float _kp, _kd, _ki;
    double _previous_e, _ie;
    float _max_ie;
    long _previous_t;
    void Compute();

    
  private:
    
    bool ReachedMaxSpeed();

    PID _controller;

    double _ctrlCmd;
    double _previousPos;
    double _lastTime;

    double _maxSpeed;
    double _maxNegSpeed;
      
    Motor* _motor;
    Encoder* _encoder;

    void(* resetFunc) (void) = 0;
};


SingleJoint::SingleJoint(Motor* motor, Encoder* enc, double maxSpeed, double maxNegSpeed, 
  unsigned int timePeriodCom=1000, double kp=1, double ki=0, double kd=0)
  : Joint(timePeriodCom), _motor(motor), _encoder(enc), _controller(&actualVel, &_ctrlCmd, &velSetpoint, kp, ki, kd, DIRECT)
{   
  _maxSpeed = maxSpeed;
  _maxNegSpeed = maxNegSpeed;

  if (enc != NULL)
    _closedLoopCtrl = true;

  _ctrlCmd = 0;
  _cmd = 0;
  velSetpoint = 0;

  _controller.SetMode(AUTOMATIC);
  _controller.SetSampleTime(1);
  _controller.SetOutputLimits(-255,255);

  _lastTime = millis();

  _kp = kp;
  _kd = kd;
  _ki = ki;
  _previous_t = 0;
}


void SingleJoint::setGains(double kp, double ki, double kd)
{
  _controller = PID(&actualVel, &_ctrlCmd, &velSetpoint, kp, ki, kd, DIRECT);
  _controller.SetMode(AUTOMATIC);
  _controller.SetOutputLimits(-255,255);
}


void SingleJoint::CalculateActVel()
{
  if (_closedLoopCtrl) 
  {
    double t = millis();
    double dt = t - _lastTime;

    /*
    Serial.println("----------");
    Serial.print(millis());
    Serial.print(" : ");
    Serial.println(dt);
    */
    
    if (dt > 0.001)
    {
      _lastTime = t;
  
      double pos = _encoder->get();
      actualVel = ((pos - _previousPos) / dt) * 1000;
      _previousPos = pos;
    }
  }
  else
    actualVel = 0.0;
}

bool SingleJoint::ReachedMaxSpeed()
{
  if (abs(_cmd) > 250)
    return true;
  return false;
}



void SingleJoint::Compute()
{
  double t = millis();
  double dt = t - _previous_t;
  double error = velSetpoint - actualVel; 
 
  
  if (dt > 0) 
  {
    //_ctrlCmd = 0;
    // KP
    _ctrlCmd = _kp*error;
    
    // KD
    if (_kd != 0)
    {
      double d_e = (error-_previous_e)/dt;
      _ctrlCmd += _kd*d_e;
    }

    // KI
    if (_ki != 0)
    {
      _ie += error*dt;
      _ctrlCmd += _ki*_ie;
    }
    
    _previous_e = error;
    _previous_t = t;
  }
 
}




void SingleJoint::JointLoop()
{ 

  //pid if encoder not null else direct command
  //CalculateActVel();

  Serial.print("actualVel:");
  Serial.print(actualVel);
  
  
  if (_closedLoopCtrl) 
  {
    _controller.Compute();
    //Compute();
    //if (_ctrlCmd > 0.02)
    _cmd += _ctrlCmd;

  }
  else
    _cmd = velSetpoint; // takes in input -1 to 1 values

  
  Serial.print(",");
  Serial.print("_cmd:");
  Serial.println(_cmd);
  
  
  if (_cmd > 255)
    _cmd = 255;
  if (_cmd < -255)
    _cmd = -255;

  /*
  AxLimits();
  if (!_bypassComm)
    CheckForComm();
  _bypassComm = false;
  */

  if (isnan(_cmd))
    resetFunc();

  _motor->set_speed_pwm(_cmd);//_cmd);
  
  reachedMaxSpeed = ReachedMaxSpeed();

  
}

#endif

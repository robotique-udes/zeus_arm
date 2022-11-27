#ifndef _SINGLEJOINTS
#define _SINGLEJOINTS

#include "Motor.h"
#include "Joint.h"
#include "Encoder.h"
#include "RunningAverage.h"

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

    
  //private:
    
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

    RunningAverage _RunnAvrgVel;

};


SingleJoint::SingleJoint(Motor* motor, Encoder* enc, double maxSpeed, double maxNegSpeed, 
  unsigned int timePeriodCom=1000, double kp=1, double ki=0, double kd=0)
  : Joint(timePeriodCom), _motor(motor), _encoder(enc), 
  _controller(&actualVel, &_ctrlCmd, &velSetpoint, kp, ki, kd, DIRECT),
  _RunnAvrgVel(20)
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

  _lastTime = 0;

  _RunnAvrgVel.clear();

}


void SingleJoint::setGains(double kp, double ki, double kd)
{
  _controller = PID(&actualVel, &_ctrlCmd, &velSetpoint, kp, ki, kd, DIRECT);
  _controller.SetMode(AUTOMATIC);
  _controller.SetOutputLimits(-255,255);
  _controller.SetSampleTime(1);
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
    Serial.println(dt);*/
    
    
    if (dt > 1)
    {
      _lastTime = t;

      double pos = _encoder->get();

      double vel = ((pos - _previousPos) / dt) * 1000;
      _RunnAvrgVel.addValue(vel);
      actualVel = _RunnAvrgVel.getAverage();


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



void SingleJoint::JointLoop()
{ 

  //Make sur its called as fast as it can go
  //CalculateActVel();

  Serial.print("actualVel:");
  Serial.print(actualVel,6);

  Serial.print(",");
  Serial.print("setpoint:");
  Serial.println(velSetpoint);


  
  if (_closedLoopCtrl) 
  {
    _controller.Compute();
    _cmd += _ctrlCmd;

  }
  else
    _cmd = velSetpoint; // takes in input -1 to 1 values
  
  if (_cmd > 255)
    _cmd = 255;
  if (_cmd < -255)
    _cmd = -255;

  
  AxLimits();
  if (!_bypassComm)
    CheckForComm();
  _bypassComm = false;


  if (isnan(_cmd))
    resetFunc();

  _motor->set_speed_pwm(_cmd);
  
  reachedMaxSpeed = ReachedMaxSpeed();

  
}

#endif

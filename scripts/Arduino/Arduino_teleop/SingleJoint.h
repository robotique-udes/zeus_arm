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
  _controller.SetOutputLimits(-255,255);

  _lastTime = millis();
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
      Serial.println("*");
      _lastTime = t;
  
      double pos = _encoder->get();
      //Serial.println(pos);
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

void SingleJoint::JointLoop()
{ 

  //pid if encoder not null else direct command
  //CalculateActVel();

  Serial.print("Variable_1:");
  Serial.print(actualVel);
  Serial.print(",");
  

  //Serial.print("Vel:");
  //Serial.println(actualVel);

  if (_closedLoopCtrl) 
  {
    _controller.Compute();
    if (_ctrlCmd > 0.02)
      _cmd += _ctrlCmd;

  }
  else
    _cmd = velSetpoint; // takes in input -1 to 1 values


  Serial.print("Variable_2:");
  Serial.println(_ctrlCmd);
  
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

/*
  Serial.print("Cmd2:");
  Serial.println(_cmd);
*/

  if (isnan(_cmd))
    resetFunc();
    
  _motor->set_speed_pwm(_cmd);
  
  reachedMaxSpeed = ReachedMaxSpeed();

  
}

#endif

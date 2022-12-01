#ifndef _MOTORS
#define _MOTORS


#include "CytronMotorDriver.h"
#include <Servo.h>


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
    virtual void set_speed(double relativeVel);
    virtual void set_speed_pwm(double pwm);
};

class Motor_cytron : public Motor
{
  public:
    Motor_cytron(int pin_pwm, int pin_dir);
    virtual void set_speed(double relativeVel);
    virtual void set_speed_pwm(double pwm);
  private: 
    CytronMD _motor;
};


Motor_cytron::Motor_cytron(int pin_pwm, int pin_dir)
  : _motor(PWM_DIR, pin_pwm, pin_dir)
{   
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_dir, OUTPUT);
}


void Motor_cytron::set_speed(double relativeVel)
{
  //Scale the setpoint in -255 to 255
  double cmd = MapCommand(relativeVel, 1., 255.0); 
  
  _motor.setSpeed(cmd);
}


void Motor_cytron::set_speed_pwm(double pwm)
{
  if (pwm > 255)
    pwm = 255;
  else if (pwm < -255)
    pwm = -255;
  
  _motor.setSpeed((int)pwm);
}




class Motor_talon : public Motor
{
  public:
    Motor_talon(int pin_pwm);
    virtual void set_speed(double relativeVel);
    virtual void set_speed_pwm(double pwm);
    void setup();
  private: 
    Servo _talon_controller;
    int _pin_pwm;
};


Motor_talon::Motor_talon(int pin_pwm)
{   
  _pin_pwm = pin_pwm;
}

void Motor_talon::setup()
{
  _talon_controller.attach(_pin_pwm);
}

void Motor_talon::set_speed(double relativeVel)
{

  //Scale between 0 and 180;
  double cmd = relativeVel * 90 + 90;

  if (cmd > 180) cmd = 180;
  if (cmd < 0) cmd = 0;

  if (cmd==90)
    _talon_controller.write(0);
  else
    _talon_controller.write(cmd);
}

void Motor_talon::set_speed_pwm(double pwm)
{
  
  double cmd_rel = MapCommand(pwm, 255, 1);
  set_speed(cmd_rel);
  
}

#endif

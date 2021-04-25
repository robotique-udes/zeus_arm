/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "CytronMotorDriver.h"
#include <ACE128.h>
#include <ACE128map87654321.h>

// Motor pins
#define dirPinm1 7
#define pwmPinm1 6


// Encoder pins
#define ACE_ADDR 0x38 

// Motor object
CytronMD motor1(PWM_DIR, pwmPinm1, dirPinm1); 

// Robot state
ACE128 myACE(ACE_ADDR, (uint8_t*)encoderMap_87654321,0);
int8_t pos;
int8_t old_pos = 127;

// Control variables
unsigned long current_time,last_time;
double set_point;
double elapsed_time;
double error, last_error;
double curr_speed;
std_msgs::Float32 velocity;
double cum_error, rate_error;
double kp = 2;
double ki = 5;
double kd = 0;

 
// Ros handle
ros::NodeHandle nh;
ros::Publisher state("/zeus_arm/joint_1_state", &velocity);



void setGains( const std_msgs::Float32MultiArray& gains_msg){
  // convert speed command to pwm signal
  kp = gains_msg.data[0];
  ki = gains_msg.data[1];
  kd = gains_msg.data[2];
  } 

void messageCb( const std_msgs::Float32& cmd_msg){
  current_time = millis();
  elapsed_time = double(current_time - last_time);

  // Calculate error
  curr_speed = currentSpeed(elapsed_time);
  error = double(cmd_msg.data) - curr_speed ;

  // Calculate integral
  cum_error += error * elapsed_time;

  // Calculate derivative
  rate_error = (error - last_error)/ elapsed_time;

  int out = int(kp * error + ki * cum_error + kd * rate_error);

  // Check limits
  if (out > 255){
    out = 255;
  }
  if (out < 0){
    out = 0;
  }

  // Set command
  motor1.setSpeed(out);

  // Publish state
  velocity.data = curr_speed;
  state.publish( &velocity );

  last_time = current_time;
  last_error = error;
  }


double currentSpeed(unsigned long elapsed_time)
{
  double velocity;
  pos = myACE.pos()* (360.0/128.0) * (M_PI/180.0);
  velocity = double((pos - old_pos)/elapsed_time);
  
  return velocity;
}


ros::Subscriber<std_msgs::Float32> sub("/zeus_arm/joint_1_velocity_controller/command", &messageCb );
ros::Subscriber<std_msgs::Float32MultiArray> gains("/zeus_arm/gains", &setGains );



void setup() {
  Serial.begin(57600);
  myACE.begin();
  nh.initNode();
  nh.advertise(state);
  nh.subscribe(sub);
  nh.subscribe(gains);
  set_point = 0;
}

void loop() {
  nh.spinOnce();
}

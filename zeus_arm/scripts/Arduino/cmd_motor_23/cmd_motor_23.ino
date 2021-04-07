/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "CytronMotorDriver.h"


#define dirPinm1 4
#define pwmPinm1 3
#define dirPinm2 7
#define pwmPinm2 6

CytronMD motor1(PWM_DIR, pwmPinm1, dirPinm1);  
CytronMD motor2(PWM_DIR, pwmPinm2, dirPinm2); 

ros::NodeHandle nh;

// Define stepper motor connections:
float posToGo = 0.0;
int motor = 0;

void messageCb( const std_msgs::Float32MultiArray& cmd_msg){
  posToGo = map(cmd_msg.data[1], -1.0, 1.0, -255.0, 255.0);
  motor = cmd_msg.data[0]; 
  posToGo = int(posToGo); 
  }  
  
ros::Subscriber<std_msgs::Float32MultiArray> sub("/zeus_arm/joint_2_velocity_controller/command", &messageCb );

void setup() {
  // Declare pins as output:
  pinMode(dirPinm1, OUTPUT);
  pinMode(pwmPinm1, OUTPUT);
  pinMode(dirPinm2, OUTPUT);
  pinMode(pwmPinm2, OUTPUT);
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  if(motor == 2){
    motor1.setSpeed(posToGo);
  }
  else if(motor == 3){
    motor2.setSpeed(posToGo);
  }
  else {
    ;
  }
  nh.spinOnce();
  delay(1);
}

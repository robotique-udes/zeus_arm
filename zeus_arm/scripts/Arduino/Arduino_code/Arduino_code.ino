/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "CytronMotorDriver.h"
#include <Servo.h>

// J1
#define j1Pin 7
Servo motor1;

// J2
#define dirPinm2 31
#define pwmPinm2 2

// J3
#define dirPinm3 35
#define pwmPinm3 3

// J4
#define dirPinm4 39
#define pwmPinm4 4

// J5
#define dirPinm5 43
#define pwmPinm5 5

// Gripper
#define dirPinG 47
#define pwmPinG 6

CytronMD motor2(PWM_DIR, pwmPinm2, dirPinm2);  
CytronMD motor3(PWM_DIR, pwmPinm3, dirPinm3); 
CytronMD motor4(PWM_DIR, pwmPinm4, dirPinm4);  
CytronMD motor5(PWM_DIR, pwmPinm5, dirPinm5); 
CytronMD gripper(PWM_DIR, pwmPinG, dirPinG);
float map_cmd(float x, float in_min, float in_max, float out_min, float out_max);

ros::NodeHandle nh;
//
//static char outstr[15];
int vel_cmd_1 = 90;
int vel_cmd_2 = 0;
int vel_cmd_3 = 0;
int vel_cmd_4 = 0;
int vel_cmd_5 = 0;


void messageCb( const std_msgs::Float64MultiArray& cmd_msg){
  vel_cmd_1 = 180 - int(map_cmd(cmd_msg.data[0], -1.0, 1.0, 0, 180));
  vel_cmd_2 = int(map_cmd(cmd_msg.data[1], -1.0, 1.0, -255.0, 255.0)); 
  vel_cmd_3 = int(map_cmd(cmd_msg.data[2], -1.0, 1.0, -255.0, 255.0)); 
  vel_cmd_4 = int(map_cmd(cmd_msg.data[3], -1.0, 1.0, -255.0, 255.0)); 
  vel_cmd_5 = int(map_cmd(cmd_msg.data[4], -1.0, 1.0, -255.0, 255.0));
//  dtostrf(vel_cmd_5,7,3,outstr); 
//  nh.loginfo(outstr);
  }  
  
ros::Subscriber<std_msgs::Float64MultiArray> sub("/zeus_arm/joint_commands", &messageCb );

float map_cmd(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Declare pins as output:
  pinMode(dirPinm2, OUTPUT);
  pinMode(pwmPinm2, OUTPUT);
  pinMode(dirPinm3, OUTPUT);
  pinMode(pwmPinm3, OUTPUT);
  pinMode(dirPinm4, OUTPUT);
  pinMode(pwmPinm4, OUTPUT);
  pinMode(dirPinm5, OUTPUT);
  pinMode(pwmPinm5, OUTPUT);

  // Declare servo
  motor1.attach(j1Pin);
  

  // Port serie
  Serial.begin(57600);

  // ROS stuff
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  motor1.write(vel_cmd_1);
  motor2.setSpeed(vel_cmd_2);
  motor3.setSpeed(vel_cmd_3);
  motor4.setSpeed(-vel_cmd_4);
  motor5.setSpeed(vel_cmd_5);

  nh.spinOnce();
  delay(1);
}

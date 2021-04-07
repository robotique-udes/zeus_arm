/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

// Define stepper motor connections:
#define dirPin 3
#define stepPin 2
#define stepsPerRev 60000
#define stepSize 360.0/stepsPerRev
#define timeoutTime 750

float posToGo = 0.0;
float actualPos = 0.0;
float lastPosToGo = 0.0;
std_msgs::Float32 curr_pos;



void messageCb( const std_msgs::Float32& cmd_msg){
  posToGo = cmd_msg.data;  
  }  




ros::Subscriber<std_msgs::Float32> sub("/zeus_arm/joint_4_velocity_controller/command", &messageCb );
ros::Publisher state("/zeus_arm/joint_4_state", &curr_pos);

void setup() {
  // Declare pins as output:
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  Serial.begin(57600);
  Serial.setTimeout(timeoutTime);
  nh.initNode();
  nh.advertise(state);
  nh.subscribe(sub);
}

void loop() {

  curr_pos.data = actualPos;
  state.publish( &curr_pos );
    
  if (posToGo > actualPos){
    digitalWrite(dirPin, HIGH);
  }
  else{
    digitalWrite(dirPin, LOW);
  }
  if (posToGo < actualPos - 0.05 || posToGo > actualPos + 0.05){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    if (posToGo > actualPos){
      actualPos = actualPos + stepSize;
    }
    else{
      actualPos = actualPos - stepSize;
    }
    lastPosToGo = posToGo;
  }
  else{
    delay(100);
  }
  nh.spinOnce();
}

#include <ams_as5048b.h>

/********************** Includes **********************/
#include <ros.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include "CytronMotorDriver.h"
#include <PID_v1.h>
#include <ams_as5048b.h>
#include <Servo.h>
#include <Bounce2.h>
/********************** Includes **********************/


/********************** Constants **********************/
// J1
#define dirPinm1 49
#define pwmPinm1 9


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

// Encoders addresses
#define joint_1_addr 0x40
#define joint_2_addr 0x41 
#define joint_3_addr 0x42 
#define joint_4_addr 0x43

// Encoders codes
#define U_DEG 3
#define U_RAD 4

// Button
#define BUTTON_PIN 8  
/********************** Constants **********************/


/********************** Objects **********************/
// Encoders
AMS_AS5048B joint_1(joint_1_addr);
AMS_AS5048B joint_2(joint_2_addr);
AMS_AS5048B joint_3(joint_3_addr);
AMS_AS5048B joint_4(joint_4_addr);


// Motor drives
CytronMD motor1(PWM_DIR, pwmPinm1, dirPinm1); 
CytronMD motor2(PWM_DIR, pwmPinm2, dirPinm2);  
CytronMD motor3(PWM_DIR, pwmPinm3, dirPinm3); 
CytronMD motor4(PWM_DIR, pwmPinm4, dirPinm4);  
CytronMD motor5(PWM_DIR, pwmPinm5, dirPinm5); 
CytronMD gripper(PWM_DIR, pwmPinG, dirPinG);

// Encoders reset button
Bounce2::Button button = Bounce2::Button();
/********************** Objects **********************/


/********************** Parameters **********************/
static char outstr[15];


// ROS publishers
ros::NodeHandle nh;
const int velocities_msg_length = 4;
const int positions_msg_length = 4;
float velocities_data[velocities_msg_length];
float positions_data[positions_msg_length];
std_msgs::Float64MultiArray velocities;
std_msgs::Float64MultiArray positions;
ros::Publisher vel_pub("/zeus_arm/joint_velocities", &velocities);
ros::Publisher pos_pub("/zeus_arm/joint_positions", &positions);

// PID setpoints
double vel_cmd_1_setpoint = 0;
double vel_cmd_2_setpoint = 0;
double vel_cmd_3_setpoint = 0;
double vel_cmd_4_setpoint = 0;
double vel_cmd_5_setpoint = 0;
double gripper_setpoint = 0;

// PID values
double kp = 5.0;
double ki = 0.0;
double kd = 0.0;
double vel_1_error = 0.0;
double vel_2_error = 0.0;
double vel_3_error = 0.0;
double vel_4_error = 0.0;
double vel_5_error = 0.0;

// Current velocity
double vel_1;
double vel_2;
double vel_3;
double vel_4;
double vel_5;

// Current velocity
double vel_1_old;
double vel_2_old;
double vel_3_old;
double vel_4_old;
double vel_5_old;

// Velocity outputs
int vel_cmd_1;
int vel_cmd_2; 
int vel_cmd_3; 
int vel_cmd_4; 
int vel_cmd_5;
int gripper_cmd;

// Current position
double joint_1_pos; 
double joint_2_pos;
double joint_3_pos;
double joint_4_pos;
double joint_5_pos;

// Old position
double old_pos_1 = 255.0;
double old_pos_2 = 255.0;
double old_pos_3 = 255.0;
double old_pos_4 = 255.0;
double old_pos_5 = 255.0;

// Control loop periods
const unsigned long time_period_low   = 300;    // 500 Hz for internal PID loop
const unsigned long time_period_high  = 300;   // 50 Hz  for ROS communication
const unsigned long time_period_com   = 10000; // 1000 ms = max com delay (watchdog)
unsigned long time_now       = 0;
unsigned long time_last_low  = 0;
unsigned long time_last_high = 0;
unsigned long time_last_com  = 0; //com watchdog

// Max velocity values
const double max_vel_1 = 0.20;
const double max_vel_2 = 0.13;
const double max_vel_3 = 0.033;
const double max_vel_4 = 0.20;
const double max_vel_5 = 1.0;
/********************** Parameters **********************/


/********************** Functions **********************/
double MapCommand(double x, double in_min, double in_max, double out_min, double out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SetGains( const std_msgs::Float64MultiArray& gains_msg)
{
  kp = gains_msg.data[0];
  ki = gains_msg.data[1];
  kd = gains_msg.data[2];
  time_last_com = millis();
} 

void MessageCallback( const std_msgs::Float64MultiArray& cmd_msg)
{
  vel_cmd_1_setpoint = cmd_msg.data[0];
  vel_cmd_2_setpoint = cmd_msg.data[1];
  vel_cmd_3_setpoint = cmd_msg.data[2];
  vel_cmd_4_setpoint = cmd_msg.data[3];
  vel_cmd_5_setpoint = cmd_msg.data[4];
  gripper_setpoint = cmd_msg.data[5];
  time_last_com = millis();
}

double GetPosition(int joint)
{
  double joint_pos, angle_raw;
  switch(joint)
  {
    case 1:
      angle_raw = joint_1.getMovingAvgExp(U_RAD);
      if (angle_raw > M_PI){
        joint_pos = angle_raw - (2 * M_PI);
      }
      else{
        joint_pos = angle_raw;
      }
      break;
    case 2:
      angle_raw = joint_2.getMovingAvgExp(U_RAD);
      if (angle_raw > M_PI){
        joint_pos = angle_raw - (2 * M_PI);
      }
      else{
        joint_pos = angle_raw;
      }
      break;
    case 3:
      angle_raw = joint_3.getMovingAvgExp(U_RAD);
      if (angle_raw > M_PI){
        joint_pos = angle_raw - (2 * M_PI);
      }
      else{
        joint_pos = angle_raw;
      }
      break;
    case 4:
      angle_raw = joint_4.getMovingAvgExp(U_RAD);
      if (angle_raw > M_PI){
        joint_pos = angle_raw - (2 * M_PI);
      }
      else{
        joint_pos = angle_raw;
      }
      break;
  }

  return joint_pos;
}

// TODO : modify this to map to real pwm
int CommandToPwm(double command, int joint)
{
  int out;
  switch(joint)
  {
    case 1:
      out = int(MapCommand(command, -max_vel_1 , max_vel_1 , -255.0, 255.0));
      break;
    case 2:
      out = int(MapCommand(command, -max_vel_2, max_vel_2, -255.0, 255.0));
      break;
    case 3:
      out = int(MapCommand(command, -max_vel_3, max_vel_3, -255.0, 255.0));
      break;
    case 4:
      out = int(MapCommand(command, -max_vel_4, max_vel_4, -255.0, 255.0));
      break;
    case 5:
      out = int(MapCommand(command, -max_vel_5, max_vel_5, -255.0, 255.0));
      break;
    // gripper
    case 6:
      out = int(MapCommand(command, -1.0, 1.0, -255.0, 255.0));
      
  }

  return out;
}

void SetPwm(int pwm, int joint)
{
  switch(joint)
  {
    case 1:
      motor1.setSpeed(pwm);
      break;
    case 2:
      motor2.setSpeed(pwm);
      break;
    case 3:
      motor3.setSpeed(pwm);
      break;
    case 4:
      motor4.setSpeed(-pwm);
      break;
    case 5: 
      motor5.setSpeed(pwm);
      break;
    // gripper
    case 6:
      gripper.setSpeed(pwm);
  }
}

double Pid(double vel_ref, double current_velocity, int joint)
{
  double out, error;
  switch(joint)
  {
    case 1:
      error = vel_ref - current_velocity;
      vel_1_error += error * time_period_low;
      out = kp * error + ki * vel_1_error; 
      break;
    
    case 2:
      error = vel_ref - current_velocity;
      vel_2_error += error * time_period_low;
      out = kp * error + ki * vel_2_error; 
      break;
    case 3:
      error = vel_ref - current_velocity;
      vel_3_error += error * time_period_low;
      out = kp * error + ki * vel_3_error; 
      break;
    case 4:
      error = vel_ref - current_velocity;
      vel_4_error += error * time_period_low;
      out = kp * error + ki * vel_4_error; 
      break;
  }

  return out;
}

void ControlLoop()
{
  // Apply PID
  double vel_ref_1 = vel_cmd_1_setpoint;
  double vel_ref_2 = vel_cmd_2_setpoint;
  double vel_ref_3 = vel_cmd_3_setpoint;
  double vel_ref_4 = vel_cmd_4_setpoint;
  double vel_ref_5 = vel_cmd_5_setpoint;
  // Get joint positions
  joint_1_pos = GetPosition(1);
  joint_2_pos = GetPosition(2);
  joint_3_pos = GetPosition(3);
  joint_4_pos = GetPosition(4);

  if(abs(joint_2_pos- old_pos_2)< 0.008){
    joint_2_pos = old_pos_2; 
  }
  if(abs(joint_3_pos- old_pos_3)< 0.008){
    joint_3_pos = old_pos_3; 
  }
  if(abs(joint_4_pos- old_pos_4)< 0.008){
    joint_4_pos = old_pos_4; 
  }

  // Calculate velocities
  vel_1 = (joint_1_pos-old_pos_1) / (time_period_low );
  vel_2 = (joint_2_pos-old_pos_2) / (time_period_low / 1000.0 );
  vel_3 = (joint_3_pos-old_pos_3) / (time_period_low / 1000.0 );
  vel_4 = (joint_4_pos-old_pos_4) / (time_period_low / 1000.0 );

//  dtostrf(vel_4,7,3,outstr); 
//  nh.loginfo(outstr);



  double out_2 = Pid(vel_ref_2, vel_2, 2);
  double out_3 = Pid(vel_ref_3, vel_3, 3);
  double out_4 = Pid(vel_ref_4, vel_4, 4);

  vel_cmd_1 = CommandToPwm(vel_ref_1, 1);
  vel_cmd_2 = CommandToPwm(vel_ref_2, 2);
  vel_cmd_3 = CommandToPwm(vel_ref_3, 3); 
  vel_cmd_4 = CommandToPwm(vel_ref_4, 4); 
  vel_cmd_5 = CommandToPwm(vel_ref_5, 5);
  gripper_cmd = CommandToPwm(gripper_setpoint,6);

  if ((vel_cmd_2 < 0) && (joint_2_pos < -0.90)){
    vel_cmd_2 = 0;
  }

  if ((vel_cmd_4 > 0) && (joint_4_pos > 0.94)){
    vel_cmd_4 = 0;
  }

  if ((vel_cmd_4 < 0) && (joint_4_pos < -1.30)){
    vel_cmd_4 = 0;
  }
  

  // Send commands
  SetPwm(vel_cmd_1, 1);
  SetPwm(vel_cmd_2, 2);
  SetPwm(vel_cmd_3, 3);
  SetPwm(vel_cmd_4, 4);
  SetPwm(vel_cmd_5, 5);
  SetPwm(gripper_cmd,6);

  // Update time and pos
  old_pos_1 = joint_1_pos;
  old_pos_2 = joint_2_pos;
  old_pos_3 = joint_3_pos;
  old_pos_4 = joint_4_pos;
  vel_1_old = vel_1;
  vel_2_old = vel_2;
  vel_3_old = vel_3;
  vel_4_old = vel_4;
}



// ROS subscribers
ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub("/zeus_arm/joint_commands", &MessageCallback );
ros::Subscriber<std_msgs::Float64MultiArray> gains_sub("/zeus_arm/gains", &SetGains );
/********************** Functions **********************/


void setup() {

  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  // Init absolute encoders
  joint_1.begin();
  joint_2.begin();
  joint_3.begin();
  joint_4.begin();

  // Set direction of rotation readings
  joint_2.setClockWise(true);
  joint_3.setClockWise(true);

  // To reset encoders
  button.attach( BUTTON_PIN ,  INPUT_PULLUP );
  button.interval(5); 
  button.setPressedState(LOW); 
  
  // Declare pins as output for linear joints
  pinMode(dirPinm1, OUTPUT);
  pinMode(pwmPinm1, OUTPUT);
  pinMode(dirPinm2, OUTPUT);
  pinMode(pwmPinm2, OUTPUT);
  pinMode(dirPinm3, OUTPUT);
  pinMode(pwmPinm3, OUTPUT);
  pinMode(dirPinm4, OUTPUT);
  pinMode(pwmPinm4, OUTPUT);
  pinMode(dirPinm5, OUTPUT);
  pinMode(pwmPinm5, OUTPUT);

  // Init ROS stuff
  nh.initNode();
  nh.advertise(vel_pub);
  nh.advertise(pos_pub);
  nh.subscribe(cmd_sub);
  nh.subscribe(gains_sub);
}

void loop() {

  time_now = millis();
  button.update();
  joint_2.updateMovingAvgExp();
  joint_3.updateMovingAvgExp();
  joint_4.updateMovingAvgExp();

  // Reset encoders 0 position if button pressed
  if ( button.pressed() ) {
    nh.loginfo("Set encoders zeros to current position");
    joint_1.setZeroReg();
    joint_2.setZeroReg();
    joint_3.setZeroReg();
    joint_4.setZeroReg();
  }

//  if (( time_now - time_last_com ) > time_period_com ) 
//  {
//    vel_cmd_1_setpoint  = 0.0;
//    vel_cmd_2_setpoint  = 0.0;
//    vel_cmd_3_setpoint  = 0.0;
//    vel_cmd_4_setpoint  = 0.0;
//    vel_cmd_5_setpoint  = 0.0;
//    //nh.loginfo("inside no com condition");
//  }

  if((time_now - time_last_low) > time_period_low )
  {
    ControlLoop();
    time_last_low = time_now;
  }
  
  unsigned long dt = time_now - time_last_high;
  if (dt > time_period_high ) {

    velocities_data[0] = float(vel_1);
    velocities_data[1] = float(vel_2);
    velocities_data[2] = float(vel_3);
    velocities_data[3] = float(vel_4);

    positions_data[0] = float(joint_1_pos);
    positions_data[1] = float(joint_2_pos);
    positions_data[2] = float(joint_3_pos);
    positions_data[3] = float(joint_4_pos);
    
    velocities.data        = &velocities_data[0];
    velocities.data_length = velocities_msg_length;
    vel_pub.publish( &velocities );

    positions.data        = &positions_data[0];
    positions.data_length = positions_msg_length;
    pos_pub.publish( &positions );
    
    time_last_high = time_now ;
  }
    // Process ROS Events
    nh.spinOnce();

}

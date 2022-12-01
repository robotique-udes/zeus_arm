
/********************** Includes **********************/
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

/*
 * No need to include them since its only class specific objects
  #include "CytronMotorDriver.h"
  #include <ams_as5048b.h>
*/

#include "Encoder.h"
#include "Limitswitch.h"
#include "SingleJoint.h"
#include "DifferentialJoint.h"
#include "Motor.h"
/********************** Includes **********************/


/********************** Constants **********************/
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

#define N_JOINTS 6

#define TIME_PERIOD_LOOP              25
#define TIME_PERIOD_ROS               50
#define TIME_PERIOD_COM               500     //1000 ms after that it sends 0

/******************* Declare and initialize variables ***************/

// for ros
sensor_msgs::JointState joint_state;
float joint_cmd[N_JOINTS], joint_vel[N_JOINTS];

ros::NodeHandle nh;
ros::Publisher joint_state_pub("/zeus_arm/joint_state", &joint_state);



// 3200 for joint 1 motor 
// 700 for worm gear motor (43.8:1 * 16 pulse/rot, https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/FIT0186_Web.pdf)
const int counts_per_revolution       = 700;

// ----------------- J1 ------------------
// PINS for joint 1
const int CHA_J1    = 2, CHB_J1 = 3;
const int PWM_MOTJ1 = 4, DIR_MOTJ1 = 10;
const int SWTCH_J1  = 30;
Encoder_oth* encJ1  = new Encoder_oth(CHA_J1, CHB_J1, counts_per_revolution, true);
Motor_cytron* motJ1 = new Motor_cytron(PWM_MOTJ1, DIR_MOTJ1);
SingleJoint* J1     = new SingleJoint(motJ1, NULL, TIME_PERIOD_COM, 0.5, 0, 0.002);
Limitswitch* swtchJ1 = new Limitswitch(SWTCH_J1, false);


// ----------------- J2 ------------------
const int PWM_MOTJ2 = 5, DIR_MOTJ2 = 15;
const int SWTCH_J2  = 31;
Encoder_ams* encJ2 = new Encoder_ams(0x41, 50, U_RAD);
Motor_cytron* motJ2 = new Motor_cytron(PWM_MOTJ2, DIR_MOTJ2);
SingleJoint* J2 = new SingleJoint(motJ2, NULL, TIME_PERIOD_COM, 0.5, 0, 0.02);
Limitswitch* swtchJ2 = new Limitswitch(SWTCH_J2, false);



// ----------------- J3 ------------------
const int PWM_MOTJ3 = 6, DIR_MOTJ3 = 14;
const int SWTCH_J3  = 32;
Encoder_ams* encJ3 = new Encoder_ams(0x42, 50, U_RAD);
Motor_cytron* motJ3 = new Motor_cytron(PWM_MOTJ3, DIR_MOTJ3);
SingleJoint* J3 = new SingleJoint(motJ3, NULL, TIME_PERIOD_COM, 0.5, 0, 0.02);
Limitswitch* swtchJ3 = new Limitswitch(SWTCH_J3, false);



// ----------------- J45 ------------------
// PINS for joint 4 & 5 (diff joint)
const int CHA_J45_1 = 18, CHB_J45_1 = 19;
const int PWM_MOTJ45_1 = 7, DIR_MOTJ45_1 = 16;
const int CHA_J45_2 = 20, CHB_J45_2 = 21;
const int PWM_MOTJ45_2 = 8, DIR_MOTJ45_2 = 13;
const int SWTCH_J45_1  = 33, SWTCH_J45_2 = 34;
// for talon drives
const int MOT_PIN_J45_1 = 12, MOT_PIN_J45_2 = 11;

Encoder_oth* encJ45_1 = new Encoder_oth(CHA_J45_1, CHB_J45_1, counts_per_revolution, true);
Motor_talon* motJ45_1 = new Motor_talon(MOT_PIN_J45_1);
SingleJoint* J45_1 = new SingleJoint(motJ45_1, NULL, TIME_PERIOD_COM, 0.2, 0, 0.0);

Encoder_oth* encJ45_2 = new Encoder_oth(CHA_J45_2, CHB_J45_2, counts_per_revolution, true);
Motor_talon* motJ45_2 = new Motor_talon(MOT_PIN_J45_2);
SingleJoint* J45_2 = new SingleJoint(motJ45_2, NULL, TIME_PERIOD_COM, 0.2, 0, 0.0);

Limitswitch* swtchJ45_1 = new Limitswitch(SWTCH_J45_1, false);
Limitswitch* swtchJ45_2 = new Limitswitch(SWTCH_J45_2, false);

DifferentialJoint* J45 = new DifferentialJoint(J45_1, J45_2, 0);

// ---------------- J6 --------------------
// PINS for joint 4 & 5 (diff joint)
const int PWM_MOTJ6 = 9;
Motor_talon* motJ6 = new Motor_talon(PWM_MOTJ6);
SingleJoint* J6 = new SingleJoint(motJ6, NULL, TIME_PERIOD_COM, 0.5, 0, 0.02);


// create ISR for each encoder and attach it
void ISR_EncJ1() { encJ1->modify_count(); }
void ISR_EncJ45_1() { encJ45_1->modify_count(); }
void ISR_EncJ45_2() { encJ45_2->modify_count(); }




unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;
unsigned long       time_last_low_ros = 0;


  
// Loops
void EncoderLoop()
{
  encJ1->encoder_loop();
  //encJ2->encoder_loop();
  //encJ3->encoder_loop();
  encJ45_1->encoder_loop();
  encJ45_2->encoder_loop();
}

// Calculate velocity for each joint with an oth_encoder
void VelocityLoop()
{
  // only for encoder_oth joints, the other encoder has already actual vel 
  J1->CalculateActVel();
  J45_1->CalculateActVel();
  J45_2->CalculateActVel();
}


void MotorLoop()
{
  J1->JointLoop();
  J2->JointLoop();
  J3->JointLoop();
  J45->JointLoop();
  J6->JointLoop();
}


/********************** CALLBACKS **********************/
void MessageCallback(const std_msgs::Float64MultiArray & cmd_msg)
{
  // J1
  J1->velSetpoint = cmd_msg.data[0];
  J1->UpdateLastComm();

  //J2
  J2->velSetpoint = cmd_msg.data[1];
  J2->UpdateLastComm();

  //J3
  J3->velSetpoint = cmd_msg.data[2];
  J3->UpdateLastComm();

  //J45
  J45->setVelSetpoint(cmd_msg.data[3], cmd_msg.data[4]);
  J45->UpdateLastComm();

  //J6
  J6->velSetpoint = cmd_msg.data[5];
  J6->UpdateLastComm();
}

// ROS subscribers
ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub("/zeus_arm/joint_commands", &MessageCallback );



/********************** Arduino Loop **********************/
double starttime;

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  // attach encoder ISR
  attachInterrupt(digitalPinToInterrupt(CHA_J1), ISR_EncJ1, RISING);
  attachInterrupt(digitalPinToInterrupt(CHA_J45_1), ISR_EncJ45_1, RISING);
  attachInterrupt(digitalPinToInterrupt(CHA_J45_2), ISR_EncJ45_2, RISING);

  encJ1->setup_enc();
  //encJ2->setup_enc();
  //encJ3->setup_enc();
  encJ45_1->setup_enc();
  encJ45_2->setup_enc();

  // set ax limits 
  J2->set_ax_limit(swtchJ2, -1);
  J3->set_ax_limit(swtchJ3, 1);
  J45->set_ax_limit(swtchJ45_1, -1);
  J45->set_ax_limit(swtchJ45_2, 1);

  motJ45_1->setup();
  motJ45_2->setup();
  motJ6->setup();

  J1->velSetpoint = 0;
  J45->setVelSetpoint(0,0);

  // Init ROS stuff
  nh.initNode();

  joint_state.position_length = N_JOINTS;
  joint_state.velocity_length = N_JOINTS;

  nh.subscribe(cmd_sub);
  nh.advertise(joint_state_pub);

  Serial.println("Setup");
  starttime = millis();
}

void loop() {
  time_now = millis();

  // Encoder loop
  EncoderLoop();

  // Velocity loop
  VelocityLoop();

  // LOOOP 
  if ((time_now - time_last_low) > TIME_PERIOD_LOOP )
  {
    if (time_now - starttime > 3000)
      MotorLoop();
      
    time_last_low = time_now;
  }

  // ROS publishers
  if ((time_now - time_last_low_ros) > TIME_PERIOD_ROS )
  {
    joint_vel[0] = J1->actualVel;
    joint_vel[1] = J2->actualVel;
    joint_vel[2] = J3->actualVel;
    joint_vel[3] = J45_1->actualVel;
    joint_vel[4] = J45_2->actualVel;
    joint_vel[5] = J6->actualVel;

    joint_cmd[0] = J1->velSetpoint;
    joint_cmd[1] = J2->velSetpoint;
    joint_cmd[2] = J3->velSetpoint;
    joint_cmd[3] = J45_1->velSetpoint;
    joint_cmd[4] = J45_2->velSetpoint;
    joint_cmd[5] = J6->velSetpoint;
  
    joint_state.position = joint_cmd;
    joint_state.velocity = joint_vel;

    // Send joint state
    joint_state_pub.publish(&joint_state);

    time_last_low_ros = time_now;
  }

  // Process ROS Events
  nh.spinOnce();
  
}

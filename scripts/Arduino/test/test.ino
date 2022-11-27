
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
/********************** Includes **********************/


/********************** Constants **********************/
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

#define TIME_PERIOD_LOOP              2
#define TIME_PERIOD_COM               500     //1000 ms after that it sends 0

// PINS for joint 1
const int CHA_J1 = 2, CHB_J1 = 3;
const int PWM_MOTJ1 = 4, DIR_MOTJ1 = 10;

// 3200 for joint 1 motor 
// 700 for worm gear motor (43.8:1 * 16 pulse/rot, https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/FIT0186_Web.pdf)
const int counts_per_revolution       = 700;
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;

float joint_pos;


Encoder_oth* enc = new Encoder_oth(CHA_J1, CHB_J1, counts_per_revolution, true);
Motor_cytron* motj1 = new Motor_cytron(PWM_MOTJ1, DIR_MOTJ1);
SingleJoint* joint = new SingleJoint(motj1, enc, 1, -1.2, TIME_PERIOD_COM, 0.5, 0, 0.02);

// create ISR for each encoder and attach it
void ISR_EncJoint1() { enc->modify_count(); }

  
// Loops
void EncoderLoop()
{
  enc->encoder_loop();
}

void MotorLoop()
{
  joint->BypassComm();
  joint->JointLoop();
}

/********************** Arduino Loop **********************/
bool test = true;
double starttime;

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  // attach encoder ISR
  attachInterrupt(digitalPinToInterrupt(CHA_J1), ISR_EncJoint1, RISING);

  enc->set_zero(0);
  Serial.println("Setup");

  joint->velSetpoint = 0;
  starttime = millis();
}

void loop() {
  time_now = millis();

  // Encoder loop
  EncoderLoop();

  joint->CalculateActVel();

  // LOOOP 
  if ((time_now - time_last_low) > TIME_PERIOD_LOOP )
  {
    
    if (millis() - starttime > 3000)
      joint->velSetpoint = 0;
    
    MotorLoop();
    
    time_last_low = time_now;
  }
  
}

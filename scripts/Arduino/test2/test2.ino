
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
/********************** Includes **********************/


/********************** Constants **********************/
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

#define TIME_PERIOD_LOOP              5
#define TIME_PERIOD_COM               500     //1000 ms after that it sends 0

// 3200 for joint 1 motor 
// 700 for worm gear motor (43.8:1 * 16 pulse/rot, https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/FIT0186_Web.pdf)
const int counts_per_revolution       = 700;

// PINS for joint 1
const int CHA_J1 = 2, CHB_J1 = 3;
const int PWM_MOTJ1 = 4, DIR_MOTJ1 = 10;


// PINS for joint 4 & 5 (diff joint)
const int CHA_J45_1 = 18, CHB_J45_1 = 19;
const int PWM_MOTJ45_1 = 7, DIR_MOTJ45_1 = 16;
const int CHA_J45_2 = 20, CHB_J45_2 = 21;
const int PWM_MOTJ45_2 = 8, DIR_MOTJ45_2 = 13;

Encoder_oth* encJ45_1 = new Encoder_oth(CHA_J45_1, CHB_J45_1, counts_per_revolution, true);
Motor_cytron* motJ45_1 = new Motor_cytron(PWM_MOTJ45_1, DIR_MOTJ45_1);
SingleJoint* J45_1 = new SingleJoint(motJ45_1, encJ45_1, 1, -1.2, TIME_PERIOD_COM, 0.5, 0, 0.02);

Encoder_oth* encJ45_2 = new Encoder_oth(CHA_J45_2, CHB_J45_2, counts_per_revolution, true);
Motor_cytron* motJ45_2 = new Motor_cytron(PWM_MOTJ45_2, DIR_MOTJ45_2);
SingleJoint* J45_2 = new SingleJoint(motJ45_2, encJ45_2, 1, -1.2, TIME_PERIOD_COM, 0.5, 0, 0.02);

DifferentialJoint* J45 = new DifferentialJoint(J45_1, J45_2, 0);

unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;



// create ISR for each encoder and attach it
void ISR_EncJ45_1() { encJ45_1->modify_count(); }
void ISR_EncJ45_2() { encJ45_2->modify_count(); }

  
// Loops
void EncoderLoop()
{
  encJ45_1->encoder_loop();
  encJ45_2->encoder_loop();
}

// Calculate velocity for each joint with an oth_encoder
void VelocityLoop()
{
  J45_1->CalculateActVel();
  J45_2->CalculateActVel();
}


void MotorLoop()
{
  J45->BypassComm();
  J45->JointLoop();

}

/********************** Arduino Loop **********************/
double starttime;

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  // attach encoder ISR
  attachInterrupt(digitalPinToInterrupt(CHA_J45_1), ISR_EncJ45_1, RISING);
  attachInterrupt(digitalPinToInterrupt(CHA_J45_2), ISR_EncJ45_2, RISING);

  encJ45_1->set_zero(0);
  encJ45_2->set_zero(0);
  

  J45->setVelSetpoint(0,0);

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
    
    if (millis() - starttime > 3000)
    {
      MotorLoop();
      
      J45->setVelSetpoint(4,0);
      Serial.println(J45->jointActive);
    }
    
    
    
    time_last_low = time_now;
  }
  
}

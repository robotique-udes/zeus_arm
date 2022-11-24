
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
#include "Joint.h"
/********************** Includes **********************/


/********************** Constants **********************/
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

#define N_MOTORS 6
#define N_ENCODERS 4
#define N_LIMITSWITCH 5

#define DT_ROS                        0.1      //Must be 1/100
#define TIME_PERIOD_ROS               100      // 100 ms publishing rate loop
#define TIME_PERIOD_COM               1000     //1000 ms after that it sends 0

const int counts_per_revolution       = 3200;
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;
float joint_pos;

double previous_pos = 0, pos_pulse;

/********************** CALLBACKS **********************/

Encoder_oth* enc = new Encoder_oth(3, 2, counts_per_revolution, true);
Motor* motj1;
  
// Loops
void Encoder_loop()
{
  enc->encoder_loop();
}


/********************** Arduino Loop **********************/

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);
  
  motj1 = new Motor_cytron(5, 7);

  //enc->set_zero(0);
  
  Serial.println("Setup");
}

void loop() {
  time_now = millis();

  // Encoder loop
  Encoder_loop();


  if ((time_now - time_last_low) > TIME_PERIOD_ROS )
  {
    pos_pulse = enc->getCounter();
    Serial.print(enc->getCounter());
    Serial.print(" : velocity ");
    Serial.println((enc->pulse2pos(pos_pulse-previous_pos)/(time_now - time_last_low))*1000);
    

    previous_pos = pos_pulse;
    
    time_last_low = time_now;
    motj1->set_speed(-1);
    

    //Serial.print("actual_vel: ");
    //Serial.println(joint1.actual_vel);
  }

  
}

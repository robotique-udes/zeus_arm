
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

const int counts_per_revolution       = 6533;
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;
float joint_pos;

/********************** CALLBACKS **********************/

Encoder* enc = new Encoder_oth(19, 18, counts_per_revolution * 2, true);
Limitswitch* swtch = new Limitswitch(30, false);
Motor* motj1 = new Motor_cytron(11, 12);
Joint joint1(motj1, 0.3, 0.01, TIME_PERIOD_COM);
  
// Loops
void Encoder_loop()
{
  enc->encoder_loop();
}

void Motor_loop()
{
  joint1.vel_setpoint = 0.5;
  joint1.closed_loop_ctrl = true;
  joint1.joint_loop();
   
}



/********************** Arduino Loop **********************/

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);


  joint1.setup_calib(enc, swtch, 0.04, -1, -2.16, 30000, 100, 25, 5);
  
  

  Serial.println("Setup");
}

void loop() {
  time_now = millis();

  // Encoder loop
  Encoder_loop();

  // Motor loop
  Motor_loop();

  if ((time_now - time_last_low) > TIME_PERIOD_ROS )
  {
    double pos = enc->get();
      
    joint1.actual_vel = (pos - joint_pos)/DT_ROS;

    joint_pos = pos;
    time_last_low = time_now;

    //Serial.print("actual_vel: ");
    //Serial.println(joint1.actual_vel);
  }

  
}


/********************** Includes **********************/
#include <ros.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

/*
#include "CytronMotorDriver.h"
#include <ams_as5048b.h>
#include <Servo.h>
#include <Bounce2.h>
#include <digitalWriteFast.h>
*/

#include "Encoder.h"
#include "Limitswitch.h"
#include "Motor.h"
/********************** Includes **********************/


/********************** Constants **********************/
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

#define N_MOTORS 6
#define N_ENCODERS 4

const int counts_per_revolution       = 6533;
const unsigned long TIME_PERIOD_LOW   = 50;      //50 ms control loop
const unsigned long TIME_PERIOD_COM   = 1000;    //1000 ms after that it sends 0
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;

int calib_value = -1;
std_msgs::Bool calib_done;
std_msgs::Float64MultiArray joint_positions;

ros::NodeHandle nh;
ros::Publisher calib_done_pub("/zeus_arm/calib_done", &calib_done);
ros::Publisher pos_pub("/zeus_arm/joint_positions", &joint_positions);

// Create encoder object :
Encoder* enc_arr[N_MOTORS] = {
    new Encoder_oth(19, 18, counts_per_revolution),
    new Encoder_ams(0x40, U_RAD),
    new Encoder_ams(0x41, U_RAD),
    new Encoder_ams(0x42, U_RAD)
};

// Create motor object : 
// Motor::Motor(int pin_pwm, int pin_dir, double max_speed, 
//    double min_speed_threshold, unsigned int time_period_com = 1000)

Motor motor_arr[N_MOTORS] = {
    Motor(11, 12, 0.3, 0.1, TIME_PERIOD_COM), //J1
    Motor(9, 10, 0.5, 0.1, TIME_PERIOD_COM), //J2
    Motor(7, 8, 1, 0.1, TIME_PERIOD_COM), //J3
    Motor(5, 22, 1, 0.1, TIME_PERIOD_COM), //J4
    Motor(3, 4, 0.5, 0.1, TIME_PERIOD_COM), //J5
    Motor(2, 13, 1, 0.1, TIME_PERIOD_COM) //J6
};
        

/********************** CALLBACKS **********************/
void MessageCallback( const std_msgs::Float64MultiArray& cmd_msg)
{
  for (int i=0; i<N_MOTORS; i++)
  {
    motor_arr[i].vel_setpoint = cmd_msg.data[i];
    motor_arr[i].UpdateLastComm();
  }
}

// ROS subscribers
ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub("/zeus_arm/joint_commands", &MessageCallback );


/********************** Arduino Loop **********************/

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);
  
  // Init ROS stuff
  nh.initNode();
  nh.subscribe(cmd_sub);
  
}

void loop() {
  time_now = millis();
  
  if ((time_now - time_last_low) > TIME_PERIOD_LOW )
  {
    for (int i=0; i<N_MOTORS; i++)
    {
      motor_arr[i].motor_loop();
    }

    //Send positions
    
    
    time_last_low = time_now; 
  }
  // Process ROS Events
  nh.spinOnce();
  
}

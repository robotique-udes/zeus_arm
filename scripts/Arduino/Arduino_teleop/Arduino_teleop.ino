
/********************** Includes **********************/
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
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

#define TIME_PERIOD_ROS               50      // 100 ms publishing rate loop
#define TIME_PERIOD_COM               1000     //1000 ms after that it sends 0

const int counts_per_revolution       = 6533;
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;

// Calib all joints
bool calib_all_joints = false;
bool in_calib = false;

std_msgs::Int16 calib_value;
sensor_msgs::JointState joint_state;
float joint_pos[N_ENCODERS], joint_vel[N_ENCODERS], joint_mode[N_ENCODERS];


ros::NodeHandle nh;
ros::Publisher calib_state_pub("/zeus_arm/calibration_state", &calib_value);
ros::Publisher joint_state_pub("/zeus_arm/joint_state", &joint_state);


std_msgs::Float64 debug;
ros::Publisher debug_pub("/zeus_arm/debug", &debug);



// Create encoder objects :
Encoder* enc_arr[N_ENCODERS] = {
  new Encoder_oth(19, 18, counts_per_revolution * 2, true), //Dont forget the ratio from motor to base
  new Encoder_ams(0x41, 50, U_RAD),// J2
  new Encoder_ams(0x42, 50, U_RAD), // J3
  new Encoder_ams(0x40, 50, U_RAD) //J4
};

// Create limitswitch objects
Limitswitch* switch_arr[N_LIMITSWITCH] = {
  new Limitswitch(30, false), //joint1
  new Limitswitch(31, true), //joint2
  new Limitswitch(32, true), //joint3
  new Limitswitch(33, true), //joint4 up
  new Limitswitch(34, true) //joint4 down
};

// Create motor objects :
Motor* motor_arr[N_MOTORS] = {
  new Motor_cytron(11, 12), //joint1
  new Motor_cytron(9, 10), //joint2
  new Motor_cytron(7, 8), //joint3
  new Motor_cytron(5, 6), //joint4
  new Motor_cytron(3, 4), //joint5
  new Motor_talon(2)//new Motor_cytron(2, 13) //joint6
};

// Create the joint objects
Joint joint_arr[N_MOTORS] = {
  Joint(motor_arr[0], 0.4, 0.001, TIME_PERIOD_COM, true), //J1 (reverse motor dir in closedloop)
  Joint(motor_arr[1], 1, 0.01, TIME_PERIOD_COM), //J2
  Joint(motor_arr[2], 1, 0.01, TIME_PERIOD_COM), //J3
  Joint(motor_arr[3], 1, 0.01, TIME_PERIOD_COM), //J4
  Joint(motor_arr[4], 1, 0.01, TIME_PERIOD_COM), //J5 (opened-loop -> no setup-calib)
  Joint(motor_arr[5], 1, 0.01, TIME_PERIOD_COM) //J6 (opened-loop -> no setup-calib)
};



// Motor calibration setup
void setup_motor_calib()
{
  // Calibration
  joint_arr[0].setup_calib(enc_arr[0], switch_arr[0], 0.20, -1, -2.16, 60000, 1., 0.0, 0.0);
  joint_arr[1].setup_calib(enc_arr[1], switch_arr[1], 0.40, -1, -0.07, 60000, 1, 0, 0);
  joint_arr[2].setup_calib(enc_arr[2], switch_arr[2], 0.65, 1, -0.90, 60000, 1, 0, 0);
  joint_arr[3].setup_calib(enc_arr[3], switch_arr[3], 0.75, -1, 0.85, 50000, 1, 0, 0);

  // Axlimits
  joint_arr[1].set_ax_limit(switch_arr[1], -1);
  joint_arr[2].set_ax_limit(switch_arr[2], 1);
  joint_arr[3].set_ax_limit(switch_arr[3], -1);
  joint_arr[3].set_ax_limit(switch_arr[4], 1);
}

/********************** CALLBACKS **********************/
void MessageCallback(const std_msgs::Float64MultiArray & cmd_msg)
{
  for (int i = 0; i < N_MOTORS; i++)
  {
    joint_arr[i].vel_setpoint = cmd_msg.data[i];
    joint_arr[i].UpdateLastComm();

    //***************** TO BE CHANGED ************************
    if (i<N_ENCODERS)
      joint_arr[i].closed_loop_ctrl = true;
  }
}

void CalibCallback(const std_msgs::Int16 & calib_cmd)
{
  calib_value.data = calib_cmd.data;
}

void EstopCallback(const std_msgs::Bool & estop)
{
  for (int i=0; i<N_MOTORS; i++)
  {
    joint_arr[i].estop_soft = estop.data;
  }
}

// ROS subscribers
ros::Subscriber<std_msgs::Float64MultiArray> cmd_sub("/zeus_arm/joint_commands", &MessageCallback );
ros::Subscriber<std_msgs::Int16> calib_sub("/zeus_arm/calib_cmd", &CalibCallback );
ros::Subscriber<std_msgs::Bool> estop_sub("/zeus_arm/estop", &EstopCallback );


// Loops
void Encoder_loop()
{
  for (int i = 0; i < N_ENCODERS; i++)
    enc_arr[i]->encoder_loop();
}

void Motor_loop()
{
  for (int i = 0; i < N_MOTORS; i++){
    joint_arr[i].joint_loop();
    //break;
  }
}


void Calib_loop()
{
  // This loop is to call the right motor calib depending on what cmd is sent
  if (calib_value.data >= 0)
  {
    if (calib_value.data == 0)
    {
      calib_all_joints = true;
      calib_value.data = N_ENCODERS;
    }
    else if (calib_value.data > 0 && calib_value.data <= N_ENCODERS)
    {
      if (!in_calib)
      {
        joint_arr[calib_value.data-1].StartCalib();
        in_calib = true;
      }
      else
      { // If calib of motor is done
        if (!joint_arr[calib_value.data-1].start_calib)
        {
          in_calib = false;
          if (calib_value.data == 1 || !calib_all_joints)
            {
              calib_value.data = -1;
              calib_all_joints = false;
            }
          else
            calib_value.data -= 1;
        }
      }
    }
  }

}

/********************** Arduino Loop **********************/

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  calib_value.data = -1;

  joint_state.position_length = N_ENCODERS;
  joint_state.velocity_length = N_ENCODERS;
  joint_state.effort_length = N_ENCODERS;

  // Init ROS stuff
  nh.initNode();
  
  nh.subscribe(cmd_sub);
  nh.subscribe(calib_sub);
  nh.subscribe(estop_sub);
  
  nh.advertise(calib_state_pub);
  nh.advertise(joint_state_pub);

  nh.advertise(debug_pub);

  // Setup encoders
  for (int i = 0; i < N_ENCODERS; i++)
    enc_arr[i]->setup_enc();

  // Setup motor calib
  setup_motor_calib();

  Serial.println("Setup");
}


void loop() {
  time_now = millis();

  // Calib loop
  Calib_loop();

  // Encoder loop
  Encoder_loop();

  // Motor loop
  Motor_loop();

  // Low level loop
  if ((time_now - time_last_low) > TIME_PERIOD_ROS )
  {
    for (int i = 0; i < N_ENCODERS; i++)
    {     
      joint_vel[i] = joint_arr[i].actual_vel;
      joint_pos[i] = joint_arr[i].actual_pos;
      joint_mode[i] = joint_arr[i].closed_loop_ctrl;
    }
    
    joint_state.position = joint_pos;
    joint_state.velocity = joint_vel;
    joint_state.effort = joint_mode;
    
    // Send joint state
    joint_state_pub.publish(&joint_state);

    // Send calib state
    calib_state_pub.publish(&calib_value);

    debug.data = joint_arr[0].debug;
    debug_pub.publish(&debug);
    

    time_last_low = time_now;
  }

  // Process ROS Events
  nh.spinOnce();
  
}

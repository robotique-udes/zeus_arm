
/********************** Includes **********************/
#include <ros.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "CytronMotorDriver.h"
#include <PID_v1.h>
#include <ams_as5048b.h>
#include <Servo.h>
#include <Bounce2.h>
#include <digitalWriteFast.h>
/********************** Includes **********************/


/********************** Constants **********************/
const unsigned long TIME_PERIOD_LOW   = 100;      //50 ms control loop
const unsigned long TIME_PERIOD_COM   = 1000;    //1000 ms after that it sends 0
unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;

ros::NodeHandle nh;


/********************** Functions **********************/
double MapCommand(double x, double in_max, double out_max)
{
  return x*abs(in_max)*abs(out_max);
}

/********************** Classes **********************/
class Motor
{
  public:
    Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold);

    void UpdateLastComm();
    void CheckForComm();
    void SendCmd();
    double vel_setpoint;
    
  private:
    int   _pin_pwm;
    int   _pin_dir;

    double _max_speed;
    double _min_speed_threshold;
    
    
    CytronMD _motor;

    unsigned long _time_last_com; //comm watchdog
    
};


Motor::Motor(int pin_pwm, int pin_dir, double max_speed, double min_speed_threshold)
  : _motor(PWM_DIR, pin_pwm, pin_dir)
{   
  _pin_pwm = pin_pwm;
  _pin_dir = pin_dir;

  _max_speed = max_speed;
  _min_speed_threshold = min_speed_threshold;

  _time_last_com = 0.0;
  
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_dir, OUTPUT);

  vel_setpoint = 0.0; //Goes from -1.0 to 1

}

void Motor::UpdateLastComm()
{ 
  _time_last_com = millis();
}

void Motor::CheckForComm()
{
  if (millis() - _time_last_com > TIME_PERIOD_COM)
    vel_setpoint = 0.0;
    //Serial.println("Nocomm");
}

void Motor::SendCmd()
{
  CheckForComm();
  
  //Keep only the sign
  if (abs(vel_setpoint) > 1)
    vel_setpoint = abs(vel_setpoint)/vel_setpoint; 

  //Scale the setpoint in -255 to 255
  double pwm = MapCommand(vel_setpoint, _max_speed, 255.0); 

  //If cmd is too small just send 0
  if (abs(pwm) < _min_speed_threshold) 
    pwm = 0; 

  //Serial.println(pwm);
  _motor.setSpeed(pwm);
}

/********************** CALLBACKS **********************/

Motor motor1(2, 1, 0.7, 0.1);
Motor motor2(3, 4, 0.5, 0.1);
Motor motor3(5, 6, 0.5, 0.1);
Motor motor4(7, 8, 0.7, 0.1);
Motor motor5(9, 10, 0.5, 0.1);
Motor motor6(11, 12, 0.3, 0.1);
        

void MessageCallback( const std_msgs::Float64MultiArray& cmd_msg)
{
  motor1.vel_setpoint = cmd_msg.data[0];
  motor2.vel_setpoint = cmd_msg.data[1];
  motor3.vel_setpoint = cmd_msg.data[2];
  motor4.vel_setpoint = cmd_msg.data[3];
  motor5.vel_setpoint = cmd_msg.data[4];
  motor6.vel_setpoint = cmd_msg.data[5];
  
  motor1.UpdateLastComm();
  motor2.UpdateLastComm();
  motor3.UpdateLastComm();
  motor4.UpdateLastComm();
  motor5.UpdateLastComm();
  motor6.UpdateLastComm();
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
    //Serial.println("You ugly");
    motor1.SendCmd();
    motor2.SendCmd();
    motor3.SendCmd();
    motor4.SendCmd();
    motor5.SendCmd();
    motor6.SendCmd();
    
    time_last_low = time_now; 
  }
  // Process ROS Events
  nh.spinOnce();
  
}

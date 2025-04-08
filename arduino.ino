// #include <RMRC_Breakout.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h> // Include for joy_msgs
#include <Servo.h>

#include "RMRC_BreakoutV2.h"

RMRC_BreakoutV2 m1(M1M2_ADDR, MOTOR_1), m2(M1M2_ADDR, MOTOR_2), m3(M3M4_ADDR, MOTOR_3), m4(M3M4_ADDR, MOTOR_4);

#define MAXIMUM_SHAFT_RPM 294.0f
#define GEAR_RATIO 34.0f

Servo servo1 ;
Servo servo2 ;
Servo Tweezers ;
Servo servo4 ;
ros::NodeHandle nh ;

int motor_jetson = 0 ;
int motor2_jetson = 0 ;
int servo1_jetson = 0 ;
int servo2_jetson = 0 ;
int Tweezers_jetson = 0 ;
int servo4_jetson = 0 ;
sensor_msgs::Joy joy_msg;

void motor_jetson_callback(const std_msgs::Int16& msgs)
{
  motor_jetson = msgs.data;
}

void motor2_jetson_callback(const std_msgs::Int16& msgs)
{
  motor2_jetson = msgs.data;
}

void servo1_jetson_callback(const std_msgs::Int16& msgs)
{
  servo1_jetson = msgs.data;
}

void servo2_jetson_callback(const std_msgs::Int16& msgs)
{
  servo2_jetson = msgs.data;
}

void servo4_jetson_callback(const std_msgs::Int16& msgs)
{
  servo4_jetson = msgs.data;
}

void Tweezers_jetson_callback(const std_msgs::Int16& msgs)
{
  Tweezers_jetson = msgs.data;
}
// void joy_msg_callback(const sensor_msgs::Joy& msg) {
// 	joy_msg = msg; // Store joy_msgs data
// 	// Example usage of joy message data:
// 	joy_msg.axes[1];
// 	// joy_msg.buttons[index] to access buttons data
// }a

ros::Subscriber<std_msgs::Int16> motor_cmd_sub("/motor_jetson",&motor_jetson_callback);
ros::Subscriber<std_msgs::Int16> motor2_cmd_sub("/motor2_jetson",&motor2_jetson_callback);
ros::Subscriber<std_msgs::Int16> servo1_cmd_sub("/servo1_jetson",&servo1_jetson_callback);
ros::Subscriber<std_msgs::Int16> servo2_cmd_sub("/servo2_jetson",&servo2_jetson_callback);
ros::Subscriber<std_msgs::Int16> Tweezers_cmd_sub("/Tweezers_jetson",&Tweezers_jetson_callback);
ros::Subscriber<std_msgs::Int16> servo4_cmd_sub("/servo4_jetson",&servo4_jetson_callback);
// ros::Subscriber<sensor_msgs::Joy> joy_msg_sub("/gui/output/joy", &joy_msg_callback);

void setup() {
 
  Serial.begin(9600);
  nh.getHardware()->setBaud(57600);
  nh.getHardware()->setPort(&Serial1);

  nh.initNode();
  nh.subscribe(motor_cmd_sub);
  nh.subscribe(motor2_cmd_sub);
  nh.subscribe(servo1_cmd_sub);
  nh.subscribe(servo2_cmd_sub);
  nh.subscribe(Tweezers_cmd_sub);
  nh.subscribe(servo4_cmd_sub);
  // nh.subscribe(joy_msg_sub);
 
  servo1.attach(A2);
  servo2.attach(A1);
  Tweezers.attach(A0);
  servo4.attach(A3);

  m1.begin();
  m2.begin();
  m3.begin();
  m4.begin();

  // After change motor mode when uploading done you should turn off board 1 time.
  m1.setMotorMode(SPEED_AND_DUTY);
  m2.setMotorMode(SPEED_AND_DUTY);            	 
  m3.setMotorMode(SPEED_AND_DUTY);
  m4.setMotorMode(SPEED_AND_DUTY);

  m1.setDuty(0);
  m2.setDuty(0);
  m3.setDuty(0);
  m4.setDuty(0);

  m1.setEncoderResolution(374.22f);
  m2.setEncoderResolution(374.22f);
  m3.setEncoderResolution(374.22f);
  m4.setEncoderResolution(374.22f);

  m1.setMaximumRPM(MAXIMUM_SHAFT_RPM * GEAR_RATIO);
  m2.setMaximumRPM(MAXIMUM_SHAFT_RPM * GEAR_RATIO);
  m3.setMaximumRPM(MAXIMUM_SHAFT_RPM * GEAR_RATIO);
  m4.setMaximumRPM(MAXIMUM_SHAFT_RPM * GEAR_RATIO);

  m1.setKp(460.0f);
  m1.setKi(0.5f);
  m1.setKd(0.01f);
    
  m2.setKp(460.0f);
  m2.setKi(0.5f);
  m2.setKd(0.01f);

  m3.setKp(460.0f);
  m3.setKi(0.5f);
  m3.setKd(0.01f);

  m4.setKp(460.0f);
  m4.setKi(0.5);
  m4.setKd(0.01f);
}

void loop() {
  // int motor_speed = map(motor_jetson, -1, 1, 0, 180);
  Serial.println(motor_jetson);
  Serial.println(motor2_jetson);
  Serial.println(servo1_jetson);
  Serial.println(servo2_jetson);
  Serial.println(servo4_jetson);
  Serial.println(Tweezers_jetson);
  // Serial.printf("joy : %s" ,joy_msg);

  servo1.write(servo1_jetson) ;
  servo2.write(servo2_jetson) ;
  Tweezers.write(Tweezers_jetson) ;
  servo4.write(servo4_jetson) ;

  if(motor_jetson == 1){
	m1.setRPM(1000);
	m2.setRPM(1000);
	m3.setRPM(1000);
	m4.setRPM(1000);
  }
  else if(motor_jetson == -1){
	m1.setRPM(-1000);
	m2.setRPM(-1000);
	m3.setRPM(-1000);
	m4.setRPM(-1000);
  }
  else if(motor_jetson == 0){
	m1.setRPM(0);
	m2.setRPM(0);
	m3.setRPM(0);
	m4.setRPM(0);
  }

  if(motor2_jetson == 1){
	m1.setRPM(-200);
	m2.setRPM(-200);
	m3.setRPM(200);
	m4.setRPM(200);
  }
  else if(motor2_jetson == -1){
	m1.setRPM(200);
	m2.setRPM(200);
	m3.setRPM(-200);
	m4.setRPM(-200);
  }
  else if(motor2_jetson == 0){
	m1.setRPM(0);
	m2.setRPM(0);
	m3.setRPM(0);
	m4.setRPM(0);
  }
  nh.spinOnce() ;
}



#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#define SERVO_PIN0 3
#define SERVO_PIN1 5
#define SERVO_PIN2 6
#define SERVO_PIN3 10

Servo myservo0;
Servo myservo1;
Servo myservo2;
Servo myservo3;

int servo0_deg = 0;
int servo1_deg = 0;
int servo2_deg = 0;
int servo3_deg = 0;

void servo0Callback(const std_msgs::Int16& msg) { servo0_deg = msg.data; }
void servo1Callback(const std_msgs::Int16& msg) { servo1_deg = msg.data; }
void servo2Callback(const std_msgs::Int16& msg) { servo2_deg = msg.data; }
void servo3Callback(const std_msgs::Int16& msg) { servo3_deg = msg.data; }


ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> servo0_command_sub("servo0/command", &servoCallback);
ros::Subscriber<std_msgs::Int16> servo1_command_sub("servo1/command", &servo1Callback);
ros::Subscriber<std_msgs::Int16> servo2_command_sub("servo2/command", &servo2Callback);
ros::Subscriber<std_msgs::Int16> servo3_command_sub("servo3/command", &servo3Callback);


void setup() {
  nh.initNode();
  nh.subscribe(servo0_command_sub);
  nh.subscribe(servo1_command_sub);
  nh.subscribe(servo2_command_sub);
  nh.subscribe(servo3_command_sub);

  // Serial.begin(115200);
  myservo0.attach(SERVO_PIN0);
  myservo1.attach(SERVO_PIN1);
  myservo2.attach(SERVO_PIN2);
  myservo3.attach(SERVO_PIN3);

void loop() {
  nh.spinOnce();
  myservo0.write(servo0_deg);
  myservo1.write(servo1_deg);
  myservo2.write(servo2_deg);
  myservo3.write(servo3_deg);
  delay(100);
}

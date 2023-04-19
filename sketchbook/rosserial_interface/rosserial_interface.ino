#include <ros.h>
#include <std_msgs/Int16.h>
#include "servo_ros.h"

#define SERVO_PIN0_l 3
#define SERVO_PIN1_l 5
#define SERVO_PIN2_r 6
#define SERVO_PIN3_r 10

// TODO: find offset value for each motors. offset is the pwm value when motor is not rorating.
#define SERVO_OFFSET0_l 90
#define SERVO_OFFSET1_l 90
#define SERVO_OFFSET2_r 90
#define SERVO_OFFSET3_r 90

#define ANALOG_PIN_0 1

int val0;
int currentState;

ServoRos myservo0_l(SERVO_PIN0_l, SERVO_OFFSET0_l);
ServoRos myservo1_l(SERVO_PIN1_l, SERVO_OFFSET1_l);
ServoRos myservo2_r(SERVO_PIN2_r, SERVO_OFFSET2_r);
ServoRos myservo3_r(SERVO_PIN3_r, SERVO_OFFSET3_r);

void servo0Callback(const std_msgs::Int16& msg) { myservo0_l.setTargetSpeed(msg.data); }
void servo1Callback(const std_msgs::Int16& msg) { myservo1_l.setTargetSpeed(msg.data); }
void servo2Callback(const std_msgs::Int16& msg) { myservo2_r.setTargetSpeed(msg.data); }
void servo3Callback(const std_msgs::Int16& msg) { myservo3_r.setTargetSpeed(msg.data); }

ros::NodeHandle nh;
std_msgs::Int16 sensor_msg;
ros::Subscriber<std_msgs::Int16> servo1_sub("servo_controller/servo1/command", &servo0Callback);
ros::Subscriber<std_msgs::Int16> servo2_sub("servo_controller/servo2/command", &servo1Callback);
ros::Subscriber<std_msgs::Int16> servo3_sub("servo_controller/servo3/command", &servo2Callback);
ros::Subscriber<std_msgs::Int16> servo4_sub("servo_controller/servo4/command", &servo3Callback);
ros::Publisher sensor_pub("sensor", &sensor_msg);

void setup() {
  nh.initNode();
  nh.subscribe(servo1_sub);
  nh.subscribe(servo2_sub);
  nh.subscribe(servo3_sub);
  nh.subscribe(servo4_sub);

  // Serial.begin(115200);
  // myservo0_l.attach(SERVO_PIN0_l);
  // myservo1_l.attach(SERVO_PIN1_l);
  // myservo2_r.attach(SERVO_PIN2_r);
  // myservo3_r.attach(SERVO_PIN3_r);

  currentState = 0;

}

int sensor(){
  val0 = analogRead(A1);
  // Serial.println(val0);
  if(val0 == 0)
  {
    currentState = 1;
  } else
  {
    currentState = 0;
  }
  return currentState;
}

void loop() {
  currentState = sensor();
  // Serial.println(currentState);
  sensor_msg.data = val0;
  sensor_pub.publish(&sensor_msg);

  myservo0_l.update();
  myservo1_l.update();
  myservo2_r.update();
  myservo3_r.update();

  delay(50);
}
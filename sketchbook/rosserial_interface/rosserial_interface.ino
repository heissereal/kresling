#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

#define SERVO_PIN0_l 3
#define SERVO_PIN1_l 5
#define SERVO_PIN2_r 6
#define SERVO_PIN3_r 10

// TODO: find offset value for each motors. offset is the pwm value when motor is not rorating.
#define SERVO0_OFFSET 90
#define SERVO1_OFFSET 90
#define SERVO2_OFFSET 90
#define SERVO3_OFFSET 90

#define ANALOG_PIN_0 1

int val0;
int currentState;

Servo myservo0_l;
Servo myservo1_l;
Servo myservo2_r;
Servo myservo3_r;

int servo0_target = 0;
int servo1_target = 0;
int servo2_target = 0;
int servo3_target = 0;

void servo0Callback(const std_msgs::Int16& msg) { servo0_target = msg.data; }
void servo1Callback(const std_msgs::Int16& msg) { servo1_target = msg.data; }
void servo2Callback(const std_msgs::Int16& msg) { servo2_target = msg.data; }
void servo3Callback(const std_msgs::Int16& msg) { servo3_target = msg.data; }

std_msgs::Int16 sensor_msg;
std_msgs::Int16 servo0_state;
std_msgs::Int16 servo1_state;
std_msgs::Int16 servo2_state;
std_msgs::Int16 servo3_state;

ros::NodeHandle nh;
ros::Publisher sensor_pub("sensor", &sensor_msg);
ros::Publisher servo0_state_pub("servo_controller/servo0/state", &servo0_state);
ros::Publisher servo1_state_pub("servo_controller/servo1/state", &servo1_state);
ros::Publisher servo2_state_pub("servo_controller/servo2/state", &servo2_state);
ros::Publisher servo3_state_pub("servo_controller/servo3/state", &servo3_state);
ros::Subscriber<std_msgs::Int16> servo0_command_sub("servo_controller/servo0/command", &servo0Callback);
ros::Subscriber<std_msgs::Int16> servo1_command_sub("servo_controller/servo1/command", &servo1Callback);
ros::Subscriber<std_msgs::Int16> servo2_command_sub("servo_controller/servo2/command", &servo2Callback);
ros::Subscriber<std_msgs::Int16> servo3_command_sub("servo_controller/servo3/command", &servo3Callback);

void setup() {
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.advertise(servo0_state_pub);
  nh.advertise(servo1_state_pub);
  nh.advertise(servo2_state_pub);
  nh.advertise(servo3_state_pub);
  nh.subscribe(servo0_command_sub);
  nh.subscribe(servo1_command_sub);
  nh.subscribe(servo2_command_sub);
  nh.subscribe(servo3_command_sub);

  // Serial.begin(115200);
  myservo0_l.attach(SERVO_PIN0_l);
  myservo1_l.attach(SERVO_PIN1_l);
  myservo2_r.attach(SERVO_PIN2_r);
  myservo3_r.attach(SERVO_PIN3_r);

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
  nh.spinOnce();

  currentState = sensor();
  // Serial.println(currentState);
  sensor_msg.data = val0;
  sensor_pub.publish(&sensor_msg);

  servo0_state.data = SERVO0_OFFSET + servo0_target;
  servo1_state.data = SERVO1_OFFSET + servo1_target;
  servo2_state.data = SERVO2_OFFSET + servo2_target;
  servo3_state.data = SERVO3_OFFSET + servo3_target;
  servo0_state_pub.publish(&servo0_state);
  servo1_state_pub.publish(&servo1_state);
  servo2_state_pub.publish(&servo2_state);
  servo3_state_pub.publish(&servo3_state);

  myservo0_l.write(SERVO0_OFFSET + servo0_target);
  myservo1_l.write(SERVO1_OFFSET + servo1_target);
  myservo2_r.write(SERVO2_OFFSET + servo2_target);
  myservo3_r.write(SERVO3_OFFSET + servo3_target);

  delay(50);
}
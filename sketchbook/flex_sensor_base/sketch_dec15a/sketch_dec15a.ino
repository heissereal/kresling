#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int16MultiArray.h>


#define ANALOG_PIN_0 1
#define ANALOG_PIN_1 2
#define ANALOG_PIN_2 4
#define ANALOG_PIN_3 5

#define SERVO_POS_DEFAULT 180
#define SERVO_POS_MAX 0
#define SENSOR_VALUE_MIN_th 775
#define SENSOR_VALUE_MAX_th 850
#define SENSOR_VALUE_MIN_1 840
#define SENSOR_VALUE_MAX_1 930
#define SENSOR_VALUE_MIN_2 770
#define SENSOR_VALUE_MAX_2 880
#define SENSOR_VALUE_MIN_3 770
#define SENSOR_VALUE_MAX_3 860
int val0;
int val1;
int val2;
int val3;
int pos_thumb;
int pos_1;
int pos_2;
int pos_3;

std_msgs::Int16MultiArray sensor_msg;
//std_msgs::Int16 sensor1_msg;
//std_msgs::Int16 sensor2_msg;
//std_msgs::Int16 sensor3_msg;

ros::NodeHandle nh;
ros::Publisher sensor_pub("sensor", &sensor_msg);
//ros::Publisher sensor1_pub("sensor1", &sensor1_msg);
//ros::Publisher sensor2_pub("sensor2", &sensor2_msg);
//ros::Publisher sensor3_pub("sensor3", &sensor3_msg);



void setup() {
  //Serial.begin(115200);
  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(sensor_pub);
  //nh.advertise(sensor1_pub);
  //nh.advertise(sensor2_pub);
  //nh.advertise(sensor3_pub);
  sensor_msg.data = (int*)malloc(sizeof(int)*4);
  sensor_msg.data_length = 4;
}

void loop() {
  nh.spinOnce();
  val0 = analogRead(ANALOG_PIN_0);
  val1 = analogRead(ANALOG_PIN_1);
  val2 = analogRead(ANALOG_PIN_2);
  val3 = analogRead(ANALOG_PIN_3);
  //Serial.println(val0);
  pos_thumb = constrain(val0, SENSOR_VALUE_MIN_th, SENSOR_VALUE_MAX_th);
  pos_thumb = map(pos_thumb, SENSOR_VALUE_MIN_th, SENSOR_VALUE_MAX_th, 150, SERVO_POS_MAX);
  //Serial.println(pos_thumb);
  pos_1 = constrain(val1, SENSOR_VALUE_MIN_1, SENSOR_VALUE_MAX_1);
  pos_1 = map(pos_1, SENSOR_VALUE_MIN_1, SENSOR_VALUE_MAX_1, SERVO_POS_DEFAULT, SERVO_POS_MAX);
  pos_2 = constrain(val2, SENSOR_VALUE_MIN_2, SENSOR_VALUE_MAX_2);
  pos_2 = map(pos_2, SENSOR_VALUE_MIN_2, SENSOR_VALUE_MAX_2, 0, 180);
  pos_3 = constrain(val3, SENSOR_VALUE_MIN_3, SENSOR_VALUE_MAX_3);
  pos_3 = map(pos_3, SENSOR_VALUE_MIN_3, SENSOR_VALUE_MAX_3, 0, 180);

  sensor_msg.data[0] = pos_thumb;
  sensor_msg.data[1] = pos_1;
  sensor_msg.data[2] = pos_2;
  sensor_msg.data[3] = pos_3;
  sensor_pub.publish(&sensor_msg);
  //sensor1_pub.publish(&sensor1_msg);
  //sensor2_pub.publish(&sensor2_msg);
  //sensor3_pub.publish(&sensor3_msg);
  delay(100);
}

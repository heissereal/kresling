#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266 
       #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   
       #include <WiFi.h>
#endif

#include <Arduino.h>
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define WIFI_SSID         "Jinshin-LAN"    
#define WIFI_PASS         "Noaj6u0n1e"
#define APP_KEY           "accf3b7b-98d1-4fe3-9af3-29cb37448f52"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "268118c1-b1b1-4ee8-8cd4-0dfe2adab6e1-342402fc-ef97-4da1-b569-b36aec08b292"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define SWITCH_ID         "63821962b8a7fefbd64a7360"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         115200                // Change baudrate to your need



#define SERVO_PIN0 D2
#define SERVO_PIN1 D6
#define SERVO_PIN2 D7
#define SERVO_PIN3 D8
Servo myservoO;
Servo myservo1;
Servo myservo2;
Servo myservo3;

void servo0Callback(const std_msgs::Int16& msg){servo0_deg = msg.data;}
void servo1Callback(const std_msgs::Int16& msg){servo1_deg = msg.data;}
void servo2Callback(const std_msgs::Int16& msg){servo2_deg = msg.data;}
void servo3Callback(const std_msgs::Int16& msg){servo3_deg = msg.data;}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> servo0_command_sub("servo0/command", &servoCallback);
ros::Subscriber<std_msgs::Int16> servo1_command_sub("servo1/command", &servo1Callback);
ros::Subscriber<std_msgs::Int16> servo2_command_sub("servo2/command", &servo2Callback);
ros::Subscriber<std_msgs::Int16> servo3_command_sub("servo3/command", &servo3Callback);


bool myPowerState = false;

void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}


bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s (via SinricPro) \r\n", deviceId.c_str(), state ? "on" : "off");
  myPowerState = state;
  if (myPowerState == HIGH) {
    digitalWrite(D8, 1); // turn on Relais
    delay(500);
    servoObject.write(95); // turn Servo to On-position
    delay(500);
    servoObject.write(30);
    delay(500);    
    digitalWrite(D8, 0); // turn off Relais
  } else {
    digitalWrite(D8, 1); // turn on Relais
    delay(500);
    servoObject.write(95); // turn Servo to Off-position
    delay(500);
    servoObject.write(30);
    delay(500);
    digitalWrite(D8, 0); // turn off Relais
  }
  return true; // request handled properly
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];

  // set callback function to device
  mySwitch.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([]() {
    Serial.printf("Connected to SinricPro\r\n");
  });
  SinricPro.onDisconnected([]() {
    Serial.printf("Disconnected from SinricPro\r\n");
  });
  SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function
void setup() {
  nh.initNode();
  nh.subscribe(servo0_command_sub);
  nh.subscribe(servo1_command_sub);
  nh.subscribe(servo2_command_sub);
  nh.subscribe(servo3_command_sub);

  myservo0.attach(SERVO_PIN0)
  myservo1.attach(SERVO_PIN1)
  myservo2.attach(SERVO_PIN2)
  myservo3.attach(SERVO_PIN3)


  pinMode(D8, OUTPUT); // Relais
  digitalWrite(D8, HIGH);
  pinMode(D8, OUTPUT); // Servo
  servoObject.attach(D8);
  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n"); // Start serial port
  setupWiFi();
  setupSinricPro();
}

void loop() {
  SinricPro.handle();
  nh.spinOnce();


}
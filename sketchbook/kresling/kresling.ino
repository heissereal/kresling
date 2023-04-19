#include <Servo.h>

#define SERVO_PIN0_l 3

#define SERVO_PIN1_l 5

#define SERVO_PIN2_r 6

#define SERVO_PIN3_r 10

#define ANALOG_PIN_0 1

int val0;

int currentState;

 

Servo myservo0_l;

Servo myservo1_l;

Servo myservo2_r;

Servo myservo3_r;

void setup() {

  Serial.begin(115200);

  myservo0_l.attach(SERVO_PIN0_l);

  myservo1_l.attach(SERVO_PIN1_l);

  myservo2_r.attach(SERVO_PIN2_r);

  myservo3_r.attach(SERVO_PIN3_r);

  currentState = 0;

}

 

int sensor(){

  val0 = analogRead(A1);

  Serial.println(val0);

  if(val0 == 0){

    currentState = 1;

  }else{

    currentState = 0;

  }

  return currentState;

}

void loop() {

  currentState = sensor();

  Serial.println(currentState);

  if (currentState == 1){

    myservo0_l.write(0);

    myservo1_l.write(10);

    myservo2_r.write(150);

    myservo3_r.write(160);

  }else{

    myservo0_l.write(92);

    myservo1_l.write(88);

    myservo2_r.write(77);

    myservo3_r.write(85);

    //delay(10);

  }

   delay(5);

}
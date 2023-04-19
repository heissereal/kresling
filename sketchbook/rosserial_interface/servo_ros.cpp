#include "servo_ros.h"

ServoRos::ServoRos(int pin, int offset):
  pin_(pin),
  offset_(offset)
{
  Servo::attach(pin_);
  Servo::write(90);
}

void ServoRos::update(){
  if(activated_)
  {
    Servo::write(offset_ + target_speed_);
  } else
  {
    Servo::write(offset_);
  }
}

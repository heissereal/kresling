#pragma once

#include <Servo.h>

class ServoRos : public Servo
{
 public:
  ServoRos(int pin, int offset);
  ~ServoRos(){};

  void setActivateFlag(bool flag) {activated_ = flag;}
  void setTargetSpeed(int target_speed) {target_speed_ = target_speed;}
  void update();

 private:
  int pin_;
  int offset_;        // the value not to rotate the motor.
  int target_speed_;  // 0(CW_max) ~ offset_(stop) ~ 180(CCW_max)
  bool activated_;

};

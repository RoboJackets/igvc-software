#ifndef PI_MOTOR_CONTROLLER_H
#define PI_MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include "pi_motor.h"

class PiMotorController
{
 public:
  PiMotorController();

 private:
  pi_motor::PiMotor pi_motor_;
};

#endif  // PI_MOTOR_CONTROLLER_H

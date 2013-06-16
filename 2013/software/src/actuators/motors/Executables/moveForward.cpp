#include "actuators/motors/MotorDriver/MotorEncoderDriver2013.h"

int main()
{
  MotorEncoderDriver2013 driver;
  int millis = 30000;
  int speed=6.0;
  driver.setVelocities(speed, speed);
}


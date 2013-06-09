#include "actuators/motors/MotorDriver/MotorEncoderDriver2013.h"


int main()
{
  MotorEncoderDriver2013 drivertermina;
  int millis = 50000;
  int speed=1.7;
  driver.setVelocities(speed, speed, millis);
}

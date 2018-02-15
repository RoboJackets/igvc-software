#include <STSL/RJRobot.h>
#include <iostream>

using namespace std;

int main()
{
  RJRobot robot;
  cout << "Robot ready!" << endl;
  try
  {
    robot.SetFloodlight(true);
  }
  catch (exception)
  {
  }
  cout << "Robot lit and ready to party!" << endl;
  unit8_t idk = robot.LightValue();
  robot.SetMotor(MotorPort::A, 300);
  robot.SetMotor(MotorPort::B, -300);
  robot.Wait(1000);
  robot.SetMotor(MotorPort::B, 300);
  robot.Wait(1000);
  int speed = 300;
  for (int i = 0; i < 6; i++)
  {
    robot.Wait(50);
    robot.SetMotor(MotorPort::A, speed);
    speed *= -1;
    robot.SetMotor(MotorPort::B, speed);
    robot.Wait(950);
  }
  while (!robot.IsButtonPressed())
  {
    robot.Wait(50ms);
  }
  robot.StopMotors();
  return 0;
}
}

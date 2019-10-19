#include <thread>
#include <chrono>
#include "pi_motor_controller.h"

PiMotorController::PiMotorController()
{
  uint32_t motor = 1;
  pi_motor_.initMotor(1);

  std::cout << "Running motor 1 forward for 5 seconds..." << std::endl;
  pi_motor_.runMotor(1, pi_motor::motor_forward);
  pi_motor_.setSpeed(1, 255);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "Stopping." << std::endl;
  pi_motor_.runMotor(1, pi_motor::motor_release);
}

int main(int argc, char** argv)
{
  PiMotorController pi_motor_controller;

  return 0;
}

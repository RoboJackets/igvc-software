#include "pi_motor.h"
#include <pigpio.h>
#include <stdexcept>

namespace pi_motor
{
PiMotor::PiMotor()
{
  initGPIO();
  initI2C();
  initHat();
}

PiMotor::~PiMotor()
{
  if (i2c_handle_ >= 0)
  {
    i2cClose(i2c_handle_);
  }

  gpioTerminate();
}

void PiMotor::initGPIO()
{
  if (gpioInitialise() < 0)
  {
    throw std::runtime_error("gpioInitialise didn't work");
  }
}

void PiMotor::initI2C()
{
  i2c_handle_ = i2cOpen(i2c_bus, motorhat_addr, i2c_flags);

  if (i2c_handle_ < 0)
  {
    throw std::runtime_error("i2c_handle_ is invalid.");
  }
}

void PiMotor::initHat()
{
  // Setup PWM
  setAllPWM(0, 0);
  i2cWriteByteData(i2c_handle_, pwm_mode2, pwm_outdrv);
  i2cWriteByteData(i2c_handle_, pwm_mode1, pwm_allcall);

  gpioDelay(5000);

  uint32_t mode1 = i2cReadByteData(i2c_handle_, pwm_mode1) & ~pwm_sleep;
  i2cWriteByteData(i2c_handle_, pwm_mode1, mode1);

  gpioDelay(5000);

  // Set PWM frequency
  uint32_t prescale = static_cast<uint32_t>(25000000.0 / 4096.0 / pwm_frequency - 1.0);
  uint32_t old_mode = i2cReadByteData(i2c_handle_, pwm_mode1);
  uint32_t new_mode = old_mode & 0x7f | 0x10;

  i2cWriteByteData(i2c_handle_, pwm_mode1, new_mode);
  i2cWriteByteData(i2c_handle_, pwm_prescale, prescale);
  i2cWriteByteData(i2c_handle_, pwm_mode1, old_mode);
  gpioDelay(5000);
  i2cWriteByteData(i2c_handle_, pwm_mode1, old_mode | 0x80);
}

void PiMotor::setAllPWM(uint32_t on, uint32_t off)
{
  i2cWriteByteData(i2c_handle_, pwm_all_led_on_l, on & 0xff);
  i2cWriteByteData(i2c_handle_, pwm_all_led_on_h, on >> 8);
  i2cWriteByteData(i2c_handle_, pwm_all_led_off_l, off & 0xff);
  i2cWriteByteData(i2c_handle_, pwm_all_led_off_h, off >> 8);
}

void PiMotor::setPWM(uint32_t pin, uint32_t on, uint32_t off)
{
  i2cWriteByteData(i2c_handle_, pwm_all_led_on_l + 4 * pin, on & 0xff);
  i2cWriteByteData(i2c_handle_, pwm_all_led_on_h + 4 * pin, on >> 8);
  i2cWriteByteData(i2c_handle_, pwm_all_led_off_l + 4 * pin, off & 0xff);
  i2cWriteByteData(i2c_handle_, pwm_all_led_off_h + 4 * pin, off >> 8);
}

void PiMotor::setPin(uint32_t pin, uint32_t value)
{
  if (pin < 0 || pin > 15)
  {
    throw std::runtime_error("PWM pin must be between 0 and 15 inclusive.");
  }

  switch (value)
  {
    case 0:
      setPWM(pin, 0, 4096);
      break;
    case 1:
      setPWM(pin, 4096, 0);
      break;
    default:
      throw std::runtime_error("PWM pin value must be 0 or 1.");
  }
}

void PiMotor::initMotor(uint32_t motor)
{
  runMotor(motor, motor_release);
  runMotor(motor, 150);
  runMotor(motor, motor_forward);
  runMotor(motor, motor_release);
}
void PiMotor::runMotor(uint32_t motor, uint32_t command)
{
  uint32_t in1, in2;

  switch (motor)
  {
    case 1:
      in1 = pwm_m1_in1;
      in2 = pwm_m1_in2;
      break;
    case 2:
      in1 = pwm_m2_in1;
      in2 = pwm_m2_in2;
      break;
    case 3:
      in1 = pwm_m3_in1;
      in2 = pwm_m3_in2;
      break;
    case 4:
      in1 = pwm_m4_in1;
      in2 = pwm_m4_in2;
      break;
    default:
      throw std::runtime_error("Invalid motor number!");
  }

  switch (command)
  {
    case motor_forward:
      setPin(in1, 1);
      setPin(in2, 0);
      break;
    case motor_back:
      setPin(in1, 0);
      setPin(in2, 1);
      break;
    case motor_release:
      setPin(in1, 0);
      setPin(in2, 0);
      break;
    default:
      throw std::runtime_error("Invalid command.");
  }
}

void PiMotor::setSpeed(uint32_t motor, uint32_t speed)
{
  if (speed < 0 || speed > 255)
  {
    throw std::runtime_error("Speed must be between 0 and 255 inclusive.");
  }

  uint32_t pwm;
  switch (motor)
  {
    case 1:
      pwm = pwm_m1_pwm;
      break;
    case 2:
      pwm = pwm_m2_pwm;
      break;
    case 3:
      pwm = pwm_m3_pwm;
      break;
    case 4:
      pwm = pwm_m4_pwm;
      break;
    default:
      throw std::runtime_error("Unsupported motor.");
  }
  setPWM(pwm, 0, speed * 16);
}

}  // namespace pi_motor

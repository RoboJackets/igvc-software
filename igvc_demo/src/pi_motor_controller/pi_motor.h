#ifndef PI_MOTORS_H
#define PI_MOTORS_H

#include <stdint-gcc.h>

namespace pi_motor
{
constexpr uint32_t i2c_bus = 0;
constexpr uint32_t motorhat_addr = 0x60;
constexpr uint32_t i2c_flags = 0;

constexpr uint32_t pwm_mode1 = 0x00;
constexpr uint32_t pwm_mode2 = 0x01;
constexpr uint32_t pwm_prescale = 0xFE;

constexpr uint32_t pwm_restart = 0x80;
constexpr uint32_t pwm_sleep = 0x10;
constexpr uint32_t pwm_allcall = 0x01;
constexpr uint32_t pwm_invert = 0x10;
constexpr uint32_t pwm_outdrv = 0x04;

constexpr uint32_t pwm_all_led_on_l = 0xFA;
constexpr uint32_t pwm_all_led_on_h = 0xFB;
constexpr uint32_t pwm_all_led_off_l = 0xFC;
constexpr uint32_t pwm_all_led_off_h = 0xFD;

constexpr uint32_t pwm_frequency = 1600.0;

constexpr uint32_t motor_forward = 1;
constexpr uint32_t motor_back = 2;
constexpr uint32_t motor_brake = 3;
constexpr uint32_t motor_release = 4;

constexpr uint32_t pwm_m1_pwm = 8;
constexpr uint32_t pwm_m1_in2 = 9;
constexpr uint32_t pwm_m1_in1 = 10;
constexpr uint32_t pwm_m2_pwm = 13;
constexpr uint32_t pwm_m2_in2 = 12;
constexpr uint32_t pwm_m2_in1 = 11;
constexpr uint32_t pwm_m3_pwm = 2;
constexpr uint32_t pwm_m3_in2 = 3;
constexpr uint32_t pwm_m3_in1 = 4;
constexpr uint32_t pwm_m4_pwm = 7;
constexpr uint32_t pwm_m4_in2 = 6;
constexpr uint32_t pwm_m4_in1 = 5;

/**
 * Magic from this class was taken from
 * https://www.raspberrypi.org/forums/viewtopic.php?t=112415
 */
class PiMotor
{
 public:
  PiMotor();
  ~PiMotor();

  void setAllPWM(uint32_t on, uint32_t off);
  void setPWM(uint32_t pin, uint32_t on, uint32_t off);
  void setPin(uint32_t pin, uint32_t value);
  void initMotor(uint32_t motor);
  void runMotor(uint32_t motor, uint32_t command);
  void setSpeed(uint32_t motor, uint32_t speed);

 private:
  int i2c_handle_;

  void initGPIO();
  void initI2C();
  void initHat();

};
}  // namespace pi_motor

#endif  // PI_MOTORS_H

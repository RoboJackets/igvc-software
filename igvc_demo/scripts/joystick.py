try:
    from adafruit_motorkit import MotorKit
except Exception as exception:
    print(exception)

import time
import pygame

left_lr_coeff = -1
left_ud_coeff = -1
right_lr_coeff = -1
right_ud_coeff = -1

try:
    kit = MotorKit()
except:
    kit = None


def drive_motors(left: float, right: float):
    if kit:
        kit.motor1.throttle = left
        kit.motor2.throttle = right


def main():
    pygame.display.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    num_axes = joystick.get_numaxes()

    print()

    counter = 0

    while True:
        pygame.event.pump()

        left_lr = left_lr_coeff * joystick.get_axis(0)
        left_ud = left_ud_coeff * joystick.get_axis(1)

        right_lr = right_lr_coeff * joystick.get_axis(3)
        right_ud = right_ud_coeff * joystick.get_axis(4)

        print(f"\rl: ({left_ud:10.6f}, {left_lr:10.6f})\t\tr: ({right_ud:10.6f}, {right_lr:10.6f})", end="")

        drive_motors(left_ud, right_ud)
        counter += 1
        time.sleep(0.001)

        if counter == 1000:
            counter = 0
            pygame.joystick.quit()
            pygame.joystick.init()
            while pygame.joystick.get_count() <= 0:
                print("Waiting for joystick to reconnect...")
                time.sleep(1)
                pygame.event.pump()
            time.sleep(0.1)
            joystick = pygame.joystick.Joystick(0)
            joystick.init()


if __name__ == '__main__':
    main()

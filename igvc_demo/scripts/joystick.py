try:
    from adafruit_motorkit import MotorKit
except Exception as exception:
    print(exception)

import time
import os

os.environ['SDL_VIDEODRIVER'] = 'dummy'
import pygame
pygame.init()
pygame.display.set_mode((1,1))

left_lr_coeff = -1
left_ud_coeff = -1
right_lr_coeff = -1
right_ud_coeff = -1

interval = 0.02                         # Time between updates in seconds, smaller responds faster but uses more processor time
controllerLostLoops = 20                # Number of loops without any joystick events before announcing the joystick as out of range

try:
    kit = MotorKit()
    kit._pca.frequency = 256
except:
    kit = None


def drive_motors(left: float, right: float):
    if kit:
        left_zero = abs(left) < 0.1
        right_zero = abs(right) < 0.1
        if left_zero and right_zero:
            kit.motor1.throttle = None
            kit.motor3.throttle = None
        else:
            kit.motor1.throttle = left
            kit.motor3.throttle = -right


def main():
    pygame.display.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Waiting for joystick to get connected")
        while pygame.joystick.get_count() == 0:
            pygame.joystick.quit()
            pygame.joystick.init()
            time.sleep(0.5)
            pygame.event.pump()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    num_axes = joystick.get_numaxes()

    print()

    loops_without_event = 0
    controller_lost = False

    while True:
        had_event = False
        events = pygame.event.get()

        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                # A button on the joystick just got pushed down
                had_event = True
            elif event.type == pygame.JOYAXISMOTION:
                # A joystick has been moved
                had_event = True

        if had_event:
            # Reset the controller lost counter
            loops_without_event = 0
            controller_lost = False
        elif controller_lost:
            try:
                # Controller has been lost
                loops_without_event += 1
                if (loops_without_event % (controllerLostLoops / 10)) == 0:
                    del joystick
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    if pygame.joystick.get_count() < 1:
                        # Controller has been disconnected, poll for reconnection
                        print('Controller disconnected!')
                        while pygame.joystick.get_count() < 1:
                            time.sleep(interval * (controllerLostLoops / 10))
                            pygame.joystick.quit()
                            pygame.joystick.init()
                    # Grab the joystick again
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    continue
                # Skip to the next loop after the interval
                time.sleep(interval)
                continue
            except:
                pass
        else:
            # No events this loop, check if it has been too long since we saw an event
            loops_without_event += 1
            if loops_without_event > controllerLostLoops:
                # It has been too long, disable control!
                print('Controller lost!')
                controller_lost = True
                # Skip to the next loop after the interval
                time.sleep(interval)
                continue

        left_lr = left_lr_coeff * joystick.get_axis(0)
        left_ud = left_ud_coeff * joystick.get_axis(1)

        right_lr = right_lr_coeff * joystick.get_axis(3)
        right_ud = right_ud_coeff * joystick.get_axis(4)

        print(f"\rl: ({left_ud:10.6f}, {left_lr:10.6f})\t\tr: ({right_ud:10.6f}, {right_lr:10.6f})", end="")

        drive_motors(left_ud, right_ud)
        time.sleep(0.001)


if __name__ == '__main__':
    main()

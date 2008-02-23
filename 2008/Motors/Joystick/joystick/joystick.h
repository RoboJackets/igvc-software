#ifndef JOYSTICK_H
#define JOYSTICK_H

#define JOY_BUTTON_DOWN(btnNum) (joystickButtons & (1 << btnNum))

extern volatile int leftAnalogX;
extern volatile int leftAnalogY;
extern volatile int rightAnalogX;
extern volatile int rightAnalogY;
extern volatile int dPadX;
extern volatile int dPadY;
extern volatile int joystickButtons;

const char* joystick_open(void);
void joystick_close(void);

#endif
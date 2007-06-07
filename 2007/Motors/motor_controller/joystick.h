#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_
#include <stdlib.h>
#include "types.h"
#include "rjte.h"

#define JOYSTICK_X_AXIS		0x07
#define JOYSTICK_Y_AXIS		0x06

/* Get joystick position and turn it into motor velocities */
// 
//   +y
//  -x +x
//    -y
//
inline MOTORSTATES_T get_speed_from_joystick(void);


#endif /* _JOYSTICK_H_ */

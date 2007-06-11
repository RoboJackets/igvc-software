#ifndef play_h
#define play_h

#include "ptypes.h"
#if __cplusplus
#include "vision/MotorOutput.h"
#endif

/* --- Public C Functions --- */
#if __cplusplus
extern "C" {
#endif

void ConnectToRobot();
Image* GetNextFrame(void);
void ProcessTransformedFrame(void);

#if __cplusplus
}
#endif

#endif

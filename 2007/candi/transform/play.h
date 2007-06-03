#ifndef play_h
#define play_h

#include "ptypes.h"

#if __cplusplus
extern "C" {
#endif

void ConnectToRobot(char *hostname, int port);
Image* GetNextFrame(void);
void ProcessTransformedFrame(void);

#if __cplusplus
}
#endif

#endif

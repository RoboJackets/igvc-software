#ifndef _COMM_H_
#define _COMM_H_
#include "rjte.h"
#include "types.h"

/**/
inline uint8_t get_speed_from_uart(MOTORSTATES_T* motorStates);

/**/
inline void send_speed(MOTORSTATES_T motorStates, uint8_t controlMode, uint8_t error);

#endif /* _COMM_H_ */

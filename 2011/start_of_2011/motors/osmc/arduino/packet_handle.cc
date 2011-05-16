#include "packet_handle.hpp"

void genTimestamp(long * sec, long * usec)
{
	//*sec =  global_time_sec + (millis()/1000) - arduino_time_millis/1000;
	//*usec = *sec - (global_time_usec + millis()*1000 - arduino_time_millis*1000);

	*sec = 0;
	*usec = 0;
}

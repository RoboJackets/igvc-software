#ifndef STANDARD_TYPES_HPP
#define STANDARD_TYPES_HPP

#include <stdint.h>

#if 0
	#ifndef __AVR_LIBC_VERSION_STRING__
		typedef char int8;
		typedef unsigned char uint8;

		typedef short int16;
		typedef unsigned short uint16;

		typedef int int32;
		typedef unsigned int uint32;
	#else
		typedef char int8;
		typedef unsigned char uint8;

		typedef int int16;
		typedef unsigned int uint16;

		typedef long int32;
		typedef unsigned long uint32;
	#endif
#else
	typedef int8_t int8;
	typedef uint8_t uint8;

	typedef int16_t int16;
	typedef uint16_t uint16;

	typedef int32_t int32;
	typedef uint32_t uint32;
#endif
#endif

#ifndef STANDARD_TYPES_HPP
#define STANDARD_TYPES_HPP

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

#endif

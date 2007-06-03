// 32-bit host version

#ifndef _TYPES_H_
#define _TYPES_H_

typedef unsigned char u8;
typedef char s8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned int u32;
typedef int s32;

#ifndef NULL
	#define NULL 0
#endif

#ifndef TRUE
	#define TRUE (1 == 1)
#endif
#ifndef FALSE
	#define FALSE (1 == 0)
#endif

#endif // _TYPES_H_


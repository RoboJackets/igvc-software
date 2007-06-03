#ifndef ptypes_h
#define ptypes_h

/* Image type - contains height, width, and data */
struct Image {
    long x;    			//optional info
	long y;				//optional
	unsigned long width;
    unsigned long height;
    unsigned char *data;

	
};
typedef struct Image Image;

#endif

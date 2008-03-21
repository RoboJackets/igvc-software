#ifndef image_h
#define image_h

/* Image type - contains height, width, and data */
typedef struct Image_struct {
    unsigned long width;	// sizeX
    unsigned long height;	// sizeY
    /*unsigned*/ char *data;
} Image;

#endif

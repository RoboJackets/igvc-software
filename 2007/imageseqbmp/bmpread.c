#include <stdlib.h>
#include <stdio.h>
#include "bmpread.h"

// quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.  
// See http://www.dcs.ed.ac.uk/~mxr/gfx/2d/BMP.txt for more info.
int ImageLoad(char *filename, Image *image) {
    FILE *file;
    unsigned long size;                 // size of the image in bytes.
    unsigned long i;                    // standard counter.
    unsigned short int planes;          // number of planes in image (must be 1) 
    unsigned short int bpp;             // number of bits per pixel (must be 24)
    char temp;                          // temporary color storage for bgr-rgb conversion.
    
    int rowSize;
    char* rowData;
    int x, y;
    char* off1;
    char* off2;

    // make sure the file is there.
    if ((file = fopen(filename, "rb"))==NULL)
    {
	//printf("File Not Found : %s\n",filename);
	return 0;
    }
    
    // seek through the bmp header, up to the width/height:
    fseek(file, 18, SEEK_CUR);

    // read the width
    if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {
	printf("Error reading width from %s.\n", filename);
	return 0;
    }
    
    // read the height 
    if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {
	printf("Error reading height from %s.\n", filename);
	return 0;
    }
    
    // calculate the size (assuming 24 bits or 3 bytes per pixel).
    size = image->sizeX * image->sizeY * 3;

    // read the planes
    if ((fread(&planes, 2, 1, file)) != 1) {
	printf("Error reading planes from %s.\n", filename);
	return 0;
    }
    if (planes != 1) {
	printf("Planes from %s is not 1: %u\n", filename, planes);
	return 0;
    }

    // read the bpp
    if ((i = fread(&bpp, 2, 1, file)) != 1) {
	printf("Error reading bpp from %s.\n", filename);
	return 0;
    }
    if (bpp != 24) {
	printf("Bpp from %s is not 24: %u\n", filename, bpp);
	return 0;
    }
	
    // seek past the rest of the bitmap header.
    fseek(file, 24, SEEK_CUR);

    // read the data. 
    image->data = (unsigned char *) malloc(size);
    if (image->data == NULL) {
	printf("Error allocating memory for color-corrected image data");
	return 0;	
    }

    if ((i = fread(image->data, size, 1, file)) != 1) {
	printf("Error reading image data from %s.\n", filename);
	return 0;
    }

	// reverse all of the colors. (bgr -> rgb)
    for (i=0;i<size;i+=3) {
	temp = image->data[i];
	image->data[i] = image->data[i+2];
	image->data[i+2] = temp;
    }
    
    // reverse all of the rows so that rows are in top->bottom order
    rowSize = image->sizeX*3;
    rowData = malloc(rowSize);
    if (rowData == NULL) return 0;
    off1 = image->data;
    off2 = image->data + (image->sizeY-1)*rowSize;
    for (y=0; y<image->sizeY/2; y++, off1 += rowSize, off2 -= rowSize) {
    	// swap rows off1 and off2
        memcpy(rowData, off1, rowSize);
        memcpy(off1, off2, rowSize);
        memcpy(off2, rowData, rowSize);
    }
    free(rowData);
    
    // we're done.
    return 1;
}

void ImageFree(Image *image) {
	free(image->data);
}

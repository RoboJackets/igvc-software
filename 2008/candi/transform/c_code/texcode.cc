#ifndef texcode_c
#define texcode_c
#include <stdlib.h>
#include <GL/glut.h> 	// these two header files are needed to compile a program in OpenGL with GLUT
#include <stdio.h>      // Header file for standard file i/o.
#include "utilfuncs.h"
#include "texcode.h"
#include "getwhim.h"
#include "screenio.h"
static int texture[1];
Image* im3;			//texture data
#ifdef paulpedantic3
int count;
#endif
long long avga,avgb,n;
extern void* memcpy (void*,const void*,size_t);

/* put screen into graphics card*/
void NextFrame (void) {
	glTexImage2D (GL_TEXTURE_2D, 0, 3, screen->width, screen->height, 0, GL_RGB, GL_UNSIGNED_BYTE, screen->data);
}

/* Init textures & image buffer */
void texinit (void) {
	glGenTextures (1, (GLuint*) &texture[0]);
	glBindTexture (GL_TEXTURE_2D, texture[0]);  // 2d texture (x and y size)
	glTexParameteri (GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
	glTexParameteri (GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	im3= (Image*) malloc (sizeof (Image));
	im3->width=OWIDTH;
	im3->height=OHEIGHT;
	im3->data= (char*) malloc (OWIDTH*OHEIGHT*3);
	#ifdef paulpedantic
	count=0;
	#endif
	avga=0;
	avgb=0;
	n=0;
}



#endif

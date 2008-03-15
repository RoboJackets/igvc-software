#ifndef texcode_c
#define texcode_c
#include <stdlib.h>
#include <GL/glut.h> 	// these two header files are needed to compile a program in OpenGL with GLUT
#include <stdio.h>      // Header file for standard file i/o.
#include "utilfuncs.h"
#include "texcode.h"


Image* im3;			//texture data
#ifdef paulpedantic
int count;
#endif
long long avga,avgb,n;
extern void* memcpy(void*,const void*,size_t);

void packin(Image* im2);

void NextFrame(void) {
	
	Image* im2=malloc(sizeof(Image));
		 ImageLoad("1.bmp", im2);	//camera data
	
		//packin(im2);
		
	glTexImage2D(GL_TEXTURE_2D, 0, 3, im2->width, im2->height, 0, GL_RGB, GL_UNSIGNED_BYTE, im2->data);
	
}
/*
void packinold(Image* im2){
	int x,y;
	for(x=0;x<im2->width;x++){
			for(y=0;y<im2->height;y++){
				im3->data[(x+(y)*TEX_DIM)*3]=im2->data[(x+y*IWIDTH)*3];//accordioning is due to wrong width, recompile with good globals
				im3->data[(x+(y)*TEX_DIM)*3+1]=im2->data[(x+y*IWIDTH)*3+1];
				im3->data[(x+(y)*TEX_DIM)*3+2]=im2->data[(x+y*IWIDTH)*3+2];
			}
		}
}

void packin(Image* im2){
	int y;
	#define IROWCH IWIDTH*3
	#define OROWCH TEX_DIM*3
	for(y=0;y<IHEIGHT;y++){
				memcpy((void*)(im3->data+y*OROWCH),(const void*)(im2->data+y*IROWCH),(size_t)IROWCH);
			}
	#undef IROWCH
	#undef OROWCH
}*/
void texinit(void){
	glGenTextures(1, (GLuint*)&texture[0]);
    glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	im3=malloc(sizeof(Image));
	im3->width=OWIDTH;
	im3->height=OHEIGHT;
    im3->data=(char*)malloc(OWIDTH*OHEIGHT*3);
	#ifdef paulpedantic
	count=0;
	#endif
	
	NextFrame();
	//debug
	avga=0;
	avgb=0;
	n=0;
}





#endif

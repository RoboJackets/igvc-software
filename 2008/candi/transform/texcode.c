#ifndef texcode_c
#define texcode_c

Image* im3;			//texture data
#ifdef paulpedantic
int count;
#endif
long long avga,avgb,n;
extern void* memcpy(void*,const void*,size_t);

void packin(Image* im2);

void NextFrame(void) {
	
	Image* im2;
		#ifdef paulpedantic
			TRYANOTHERONE:
			im2 = (Image*) GetNextFrame();	//camera data
			if( ((im2->width)!=IWIDTH) || ((im2->height)!=IHEIGHT) ){		//do they not match?
				printf("\nWidth and height wrong x%d!!!\nW:%d  H:%d\n",++count,(int)im2->width,(int)im2->height);
				goto TRYANOTHERONE;
			}
		#else
			im2 = (Image*) GetNextFrame();	//camera data
		#endif
	
		packin(im2);
		
	glTexImage2D(GL_TEXTURE_2D, 0, 3, im3->width, im3->height, 0, GL_RGB, GL_UNSIGNED_BYTE, im3->data);
	
}

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
}
void texinit(void){
	glGenTextures(1, (GLuint*)&texture[0]);
    glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	im3=malloc(sizeof(Image));
	im3->width=TEX_DIM;
	im3->height=TEX_DIM;
    im3->data=(char*)malloc(TEX_DIM*TEX_DIM*3);
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

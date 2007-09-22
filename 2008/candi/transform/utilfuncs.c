
#include <stdlib.h>
#include <sys/time.h>
#include <GL/glut.h> 	// needed for GLUT
#include <stdlib.h>
#include <stdio.h>      // Header file for standard file i/o.


/* floats for x rotation, y rotation, z rotation */
float xrot, yrot, zrot;

/* storage for one texture  */
int texture[1];


// quick and dirty bitmap loader...for 24 bit bitmaps with 1 plane only.  
// See http://www.dcs.ed.ac.uk/~mxr/gfx/2d/BMP.txt for more info.
int ImageLoad(char *filename, Image *image) {
    FILE *file;
    unsigned long size;                 // size of the image in bytes.
    unsigned long i;                    // standard counter.
    unsigned short int planes;          // number of planes in image (must be 1) 
    unsigned short int bpp;             // number of bits per pixel (must be 24)
    char temp;                          // temporary color storage for bgr-rgb conversion.

    // make sure the file is there.
    if ((file = fopen(filename, "rb"))==NULL)
    {
	printf("File Not Found : %s\n",filename);
	return 0;
    }
    
    // seek through the bmp header, up to the width/height:
    fseek(file, 18, SEEK_CUR);

    // read the width (18->21)
    if ((i = fread(&image->width, 4, 1, file)) != 1) {
	printf("Error reading width from %s.\n", filename);
	return 0;
    }
    printf("Width of %s: %lu\n", filename, image->width);
    
    // read the height 
    if ((i = fread(&image->height, 4, 1, file)) != 1) {
	printf("Error reading height from %s.\n", filename);
	return 0;
    }
    printf("Height of %s: %lu\n", filename, image->height);
    
    // calculate the size (assuming 24 bits or 3 bytes per pixel).
    size = image->width * image->height * 3;

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
    image->data = (char *) malloc(size);
    if (image->data == NULL) {
	printf("Error allocating memory for color-corrected image data");
	return 0;	
    }

    if ((i = fread(image->data, size, 1, file)) != 1) {
	printf("Error reading image data from %s.\n", filename);
	return 0;
    }

    for (i=0;i<size;i+=3) { // reverse all of the colors. (bgr -> rgb)
	temp = image->data[i];
	image->data[i] = image->data[i+2];
	image->data[i+2] = temp;
    }
    
    // we're done.
    return 1;
}
    
// Load Bitmaps And Convert To Textures
void LoadGLTextures() {	
    // Load Texture
    Image *image1;
    
    // allocate space for texture
    image1 = (Image *) malloc(sizeof(Image));
    if (image1 == NULL) {
	printf("Error allocating space for image");
	exit(0);
    }

    if (!ImageLoad("pic.bmp", image1)) {
	exit(1);
    }        

    // Create Texture	
    glGenTextures(1, &texture[0]);
    glBindTexture(GL_TEXTURE_2D, texture[0]);   // 2d texture (x and y size)

    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); // scale linearly from mipmaps when image smaller than texture

    // 2d texture, level of detail 0 (normal), 3 components (red, green, blue), x size from image, y size from image, 
    // border 0 (normal), rgb color data, unsigned byte data, and finally the data itself.
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->width, image1->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image1->data);
}

void drawRose(void){
	
	glPushMatrix();
	glLoadIdentity ();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity ();
	glColor3f(1,.2,.2);//xrose
  	glRectd(1.0,.01,.98,-.01);
  	glColor3f(1,1,1);
	glColor3f(.2,1,.2);//yrose
  	glRectd(.01,1,-.01,.98);
  	glColor3f(1,1,1);
	glColor3f(.2,.2,1);//zrose
  	glRectd(.01,.01,-.01,-.01);
  	glColor3f(1,1,1);
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}	
inline unsigned long long nanotime(void)
{
#ifdef __i386__
     unsigned long long val;
     //asm( "statements" : output_registers : input_registers : clobbered_registers);
     //or _asm_ )"statements" : output_registers : input_registers : clobbered_registers);
    __asm__ __volatile__("rdtsc" : "=A" (val) : );
     return(val);
#else
	// This is architecture-independent but provides low granularity
	return 1000*currentTimeMicros();
#endif
}
long long currentTimeMillis() {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t * 1000) + (tv.tv_usec/1000);

   return t;
}
long long currentTimeMicros() {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t * 1000 * 1000) + (tv.tv_usec);

   return t;
}

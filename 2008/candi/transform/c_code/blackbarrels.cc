#include <stdlib.h>
#include <GL/glut.h> 	// these two header files are needed to compile a program in OpenGL with GLUT
#include <stdio.h>      // Header file for standard file i/o.
#include "utilfuncs.h"
#include "texcode.h"
#include "screenio.h"
#include "blackmain.h"

#define ESCAPE 27
#define OWIDTH 720
#define OHEIGHT 480

void keyPressed(unsigned char key, int x, int y);
int mainwindow;
int i=0;
Image out;
long long tvar;
long long tvararr[256];
unsigned char tvarindex=0;
void display ()
{
	long long tvar2;
	/* clear screen */
	glClear (GL_COLOR_BUFFER_BIT);
	
	/* get image ready */  
	blackmain();      
	
	/* create 3D flat plane to display image on */                                     
	glBegin(GL_QUADS);
		glTexCoord2f(0.f,0.f); glVertex3f(0.f		,0.f	,0);
		glTexCoord2f(1.f,0.f); glVertex3f(OWIDTH	,0.f	,0);
		glTexCoord2f(1.f,1.f); glVertex3f(OWIDTH	,OHEIGHT,0);
		glTexCoord2f(0.f,1.f); glVertex3f(0			,OHEIGHT,0);
	glEnd();
  	glBegin(GL_POINTS);
		glVertex3f(0.5f		,0.5f	,0.5f);
	glEnd();
	
	/* prepare image buffer */
  	glFlush (); // makes sure commands are executed immediately
  	
  	/* double buffering */
	glutSwapBuffers();
	   
	/* finish image buffer */
	glFinish();
	
}

/* The function called whenever a key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{
	int xx=0;
	int yy=0;
	int c=0;
	double avg=0;
	FILE *outp;
	
	/* If 's' is pressed, save everything, then kill everyting. */  
	if (key == 's') //save raw data
	{ 
	    out.data=(char*)malloc(OWIDTH*OHEIGHT*sizeof(char)*3);
	    glReadPixels(	0				,		//GLint x,
		     			0				,		//GLint y,
		     			OWIDTH			,		//GLsizei width,
		     			OHEIGHT			,		//GLsizei height,
		     			GL_BGRA			,		//GLenum format,
		     			GL_UNSIGNED_BYTE,		//GLenum type,
		     			out.data	);			//GLvoid *pixels
	    /* write data */ 					
	    outp=fopen("outraw", "wb");
	    for(xx=0;xx<OHEIGHT*OWIDTH*3;xx++){
	      fprintf(outp,"%c",*(out.data+xx));
	    }
	    fclose(outp);
		/* shut down our window */
		free(out.data);
		glutDestroyWindow(mainwindow); 
		/* exit the program...normal termination. */
		exit(0);                   
	 }
	    
	/* if any key is pressed, kill everything */
		/* shut down our window */
		glutDestroyWindow(mainwindow); 
	    /* exit the program...normal termination. */
		exit(0); 
}

void init ()
{
	texinit();							// Init textures & image buffer
	//LoadGLTextures();					// Load The Texture(s) (secodnary failsafe)
    glEnable(GL_TEXTURE_2D);			// Enable Texture Mapping
  	//glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
    glShadeModel(GL_FLAT);				// Enables Smooth Color Shading
	glClearColor (0.0, 0.0, 0.0, 0.0); 	// sets background color to black                      
	glMatrixMode(GL_PROJECTION);		// Specify matrix used
	//glLoadMatrixd(getPjMat());		// apply planar perspective transform
	//glLoadIdentity ();				// make sure OpenGL doesn't f*ck with my results
	glOrtho (.0, 720., 0, 480., -10.0, 10.0);	// creating othogonal mapping matrix
	out.width = OWIDTH;							// sets up display canvas
	out.height = OHEIGHT;						// sets up display canvas
	out.data = (char*)malloc(OWIDTH * OHEIGHT * 4);	// sets up display canvas
}

int main(int argc, char** argv)
{
	glutInit (&argc, argv); // initialization      
	glutInitDisplayMode (GLUT_DOUBLE| GLUT_RGB); // certain settings 
	glutInitWindowSize (OWIDTH, OHEIGHT); // sets wize in pixels (horizontal and vertical)
	glutInitWindowPosition (10, 10); // upper left corner position
	mainwindow=glutCreateWindow ("Transform"); // sets the window title        
	init (); // calls the init() function above
	glutDisplayFunc (display); // uses the function called 'display' for displaying 
	glutIdleFunc(display); // also display function when idle
	glutKeyboardFunc(&keyPressed); // setup keyboard interrupt
	glutMainLoop (); // run program = will execute OpenGL functions continuously if window left up
	return 0;
}



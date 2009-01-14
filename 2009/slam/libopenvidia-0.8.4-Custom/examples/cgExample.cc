/* COMPILE: 

g++ cgExample.cc  -I../include -L../ -I/usr/include/cc++ -I/usr/include/cc++2/ -ldc1394_control -lraw1394  -lstdc++ -lccext2 -lccgnu2 -lxml -lopenvidia -lpthread -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lImlib -I~/SDK/LIBS/inc -lCgGL

  */
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>
#include <Imlib.h>
#include <openvidia/openvidia32.h>
#include <iostream>
#include <stdio.h>

using namespace std;
ImlibImage *Im;
GLuint tex, oTex ;
int width = 256;
int height = 256;

FBO_Filter *filter ;

void reshape(int w, int h)
{
  glClearColor (0.0, 0.0, 0.0, 0.0);
  glViewport(0, 0, (GLsizei) w, (GLsizei) h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glFrustum(0.0, 1.0,  0.0, 1.0,   1.0,   100.0);

  gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  -1.0,   0.0, 1.0, 0.0);

  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();
  glutPostRedisplay();
}

void myIdle(){
  glutPostRedisplay();
}

void keyboard (unsigned char key, int x, int y)
{
   switch (key) {
      case 27:
         exit(0);
         break;
      default:
         break;
   }
}

void MouseFunc( int button, int state, int x, int y)
{
  switch(button) {
    case GLUT_LEFT_BUTTON :
      break;
    case GLUT_RIGHT_BUTTON :
      break;
  }
}


void render_redirect() {
  float d =  -1.0;

  //apply the texture to the filtered image. 
  // the 'false' denotes whether or not it needs to be internally
  // flipped.  in our case, we are operating on data scanned in in 
  // raster form, so we dont need to flip it.
  // if we had a second filter acting on the output of the first, we would
  // need to flip it. 
  filter->apply( tex, false);

  glClearColor(0.0, 0.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glEnable(GL_TEXTURE_RECTANGLE_NV);
  glBindTexture(GL_TEXTURE_RECTANGLE_NV, oTex) ;

  // draw the 'tex' texture containing the framebuffer rendered drawing.
  glBegin(GL_QUADS); 
    glTexCoord2f(0, height);
    glVertex3f( 0.0, 1.0, d );

    glTexCoord2f(0, 0);
    glVertex3f( 0.0, 0.0, d );

    glTexCoord2f(width, 0);
    glVertex3f( 1.0, 0.0, d );

    glTexCoord2f(width,height);
    glVertex3f( 1.0, 1.0, d );
  glEnd();

  glDisable(GL_TEXTURE_RECTANGLE_NV);
  glutSwapBuffers();
}

void load_image( char *fname)
{
   fprintf(stderr,"Opening %s\n", fname );

   ImlibData *imlib_id = Imlib_init(XOpenDisplay(":0.0") );

   Im  = Imlib_load_image(imlib_id, fname );

   if( Im == NULL ) {
     fprintf(stderr," Could not load the image\n");
     exit(0);
   }
   width = Im->rgb_width;
   height = Im->rgb_height; 
  fprintf(stderr, "Image is %d %d\n", width, height);

}


int main(int argc, char *argv[] )  {
   if( argc != 3 ) {
     fprintf(stderr, "supply 2 parameters:and a cg program filename, and an image filename.\n");
     exit(0);
   }
   load_image(argv[2]);

   glutInit( &argc, argv );
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
   glutInitWindowSize(width, height );
   glutCreateWindow(argv[0]);

   // make a texture
   glGenTextures(1, &tex);              // texture 
   glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
   glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
   glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, width,height, 0,
                GL_RGB, GL_UNSIGNED_BYTE, Im->rgb_data );
   glGenTextures(1, &oTex);              // texture 
   glBindTexture(GL_TEXTURE_RECTANGLE_NV, oTex);
   glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, width,height, 0,
                GL_RGB, GL_UNSIGNED_BYTE, NULL );

   filter = new FBO_Filter(CG_PROFILE_FP30,argv[1], oTex, width, height);

   glutDisplayFunc(render_redirect);
   glutIdleFunc(myIdle);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMouseFunc(MouseFunc);
   glutMainLoop();
   return 0;
}

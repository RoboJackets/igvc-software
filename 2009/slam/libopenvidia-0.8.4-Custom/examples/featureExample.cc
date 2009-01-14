/* COMPILE: 

g++ featureExample.cc  -I../include -L../ -I/usr/include/cc++ -I/usr/include/cc++2/ -ldc1394_control -lraw1394  -lstdc++ -lccext2 -lccgnu2 -lxml -lopenvidia -lpthread -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lImlib -I~/SDK/LIBS/inc -lCgGL

  */

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>
#include <Cg/cgGL.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <Imlib.h>
#include <vector>
#include <openvidia/openvidia32.h>


using namespace std;

GLuint tex ;
int width = 0;
int height = 0;

Dc1394 *CamSource;
featureTrack *ft;

CGprogram basicProgram;
CGprofile cgProfile;
CGcontext cgContext;

CGprogram load_cgprogram(CGprofile prof, char *name) {
  fprintf(stderr, "loading %s\n", name);
  return cgCreateProgramFromFile( cgContext, CG_SOURCE,
                                  name, prof, "FragmentProgram", 0);
}


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
  //glutPostRedisplay();
}

void showstats() {
  static int lasttime;
  static int fpscounter;
  static int fps;
  int curtime;
  curtime = time(NULL);
  if( lasttime != curtime) {
    fps=fpscounter;
    fpscounter=1;
    lasttime = curtime;
    fprintf(stderr, "fps = %d,  %f msecs\n",fps,1.0/((float)fps)*1000);

  } else {
    fpscounter++;
  }
}


void myIdle(){
  glutPostRedisplay();
}

int viewbuf=0;

void keyboard (unsigned char key, int x, int y)
{
   char keystr[2];

  keystr[0] = key;
  keystr[1] = '\0';


   switch (key) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
         //viewbuf = atoi((const char * )&key);
         viewbuf = atoi(keystr);
         break;

      case 27:
         //delete CamSource;
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

  glClearColor(0.0, 0.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glEnable(GL_TEXTURE_RECTANGLE_NV);
  glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex) ;

  //wait for a new frame to be captured.
  // this can be removed if you dont mind re-using a previous frame.  That wil let you 
  //  see a higher processing rate.
  CamSource->wait();

  //lock the image data so it is not updated while we are capturing.
  CamSource->lock();
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0,0, CamSource->width(), CamSource->height(),
                GL_RGB, GL_UNSIGNED_BYTE,CamSource->ptr());
  //free the image data so it can be updated again.
  CamSource->unlock();

  Scene s; 
  ft->getScene(tex, s);
  cerr<<"Scene with "<< s.features.size() <<" features.\n";
  // toss the scene away after we're done to avoid memory leak
  //delete(s); //segfaults.

  glBindTexture(GL_TEXTURE_RECTANGLE_NV, ft->tex[viewbuf] );
  reshape(width,height);

  //bind our simple fragment program to make sure we can see intermediate buffers.
  cgGLEnableProfile(cgProfile);
  cgGLBindProgram(basicProgram);

  glBegin(GL_QUADS); 

    glTexCoord2f(0, CamSource->height() );
    glVertex3f(1.0,1.0, d);

    glTexCoord2f(0, 0);
    glVertex3f(1.0, 0.0, d );

    glTexCoord2f(CamSource->width(), 0);
    glVertex3f(0.0, 0.0,d );

    glTexCoord2f(CamSource->width(), CamSource->height() );
    glVertex3f(0.0, 1.0, d);
  glEnd();

  glDisable(GL_TEXTURE_RECTANGLE_NV);
  cgGLDisableProfile(cgProfile);

  glutSwapBuffers();
  showstats();
}

int main(int argc, char *argv[] )  {

   int CaptureWidth = 320, CaptureHeight = 240;
   int DisplayWidth = 640, DisplayHeight = 480;
   CamSource = new Dc1394(CaptureWidth, CaptureHeight);

   width = DisplayWidth;
   height = DisplayHeight;
   glutInit( &argc, argv );
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH |GLUT_STENCIL);
   glutInitWindowSize(width, height );
   glutCreateWindow(argv[0]);


   ft = new featureTrack(CamSource->width(), CamSource->height() );

   // make a texture
   glGenTextures(1, &tex);              // texture 
   glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
   glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
   glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, CamSource->width() , 
                CamSource->height(), 0, GL_RGB, GL_UNSIGNED_BYTE,NULL );


   //start the camera capture
   CamSource->start();

   //in order to see some of the processing buffers, we need 
   // a fragment program because the floating point values wont 
   // show up on their own.  FP-basic.cg just pass texels through.
   cgProfile = CG_PROFILE_FP40;
   cgContext = cgCreateContext();
   basicProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-basic.cg");
   cgGLLoadProgram( basicProgram );

   glutDisplayFunc(render_redirect);
   glutIdleFunc(myIdle);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMouseFunc(MouseFunc);
   glutMainLoop();
   return 0;
}

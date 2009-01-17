/* COMPILE:

g++ featureExample.cc  -I../include -L../ -I/usr/include/cc++ -I/usr/include/cc++2/ -ldc1394_control -lraw1394  -lstdc++ -lccext2 -lccgnu2 -lxml -lopenvidia -lpthread -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lImlib -I~/SDK/LIBS/inc -lCgGL

  */

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <Imlib.h>
#include <openvidia/openvidia32.h>

using namespace std;

GLuint tex ;
int width = 256;
int height = 256;

Dc1394 CamSource(320,240);

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

void myIdle() {
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
    switch (button) {
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
    // this can be removed if you dont mind re-using a previous frame.
    CamSource.wait();
    //lock the image data so it is not updated while we are capturing.
    CamSource.lock();
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0,0, width,height,
                    GL_RGB, GL_UNSIGNED_BYTE,CamSource.ptr());
    //free the image data so it can be updated again.
    CamSource.unlock();

    // draw the 'tex' texture containing the framebuffer rendered drawing.
    glBegin(GL_QUADS);

    glTexCoord2f(0, height);
    glVertex3f(0.0, 0.0,d );

    glTexCoord2f(0, 0);
    glVertex3f(0.0, .5, d);

    glTexCoord2f(width, 0);
    glVertex3f(.5,.5, d);

    glTexCoord2f(width,height);
    glVertex3f(.5, 0.0, d );
    glEnd();

    glDisable(GL_TEXTURE_RECTANGLE_NV);
    glutSwapBuffers();
}

int main(int argc, char *argv[] )  {

    width = CamSource.width();
    height = CamSource.height();

    glutInit( &argc, argv );
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(width, height );
    glutCreateWindow(argv[0]);

    // make a texture
    glGenTextures(1, &tex);              // texture
    glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, width,height, 0,
                 GL_RGB, GL_UNSIGNED_BYTE,NULL );


    //start the camera capture
    CamSource.start();

    glutDisplayFunc(render_redirect);
    glutIdleFunc(myIdle);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(MouseFunc);
    glutMainLoop();
    return 0;
}

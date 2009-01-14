
#include "CVcam.h"
#include "image_buffers.h"

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


#include "vision.h"
CVcam CamSource;
Vision vp;
int trackbarVal;
void init();
void trackbarHandler(int pos);
void dovision();


using namespace std;

GLuint tex ;
featureTrack *ft;
CGprogram basicProgram;
CGprofile cgProfile;
CGcontext cgContext;
bool refreshRefScene = false;
int width, height;

void drawFeatures( Scene *currentScene );
void drawMatches( Matches &bestM ) ;

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
    if ( lasttime != curtime) {
        fps=fpscounter;
        fpscounter=1;
        lasttime = curtime;
        fprintf(stderr, "fps = %d,  %f msecs\n",fps,1.0/((float)fps)*1000);

    } else {
        fpscounter++;
    }
}


void myIdle() {
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
    case 'r' :  //request that new refernce scene be saved - will use the imediate next frame.
        refreshRefScene = true;
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


	//wait for a new frame to be captured.
    CamSource.GrabCvImage();
    dovision(); //first!!
    
    
    static Scene *refScene = NULL;

	//Scene *newScene ;

    float d =  -1.0;

    glClearColor(0.0, 0.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tex) ;



	//lock the image data so it is not updated while we are capturing.
	//CamSource->lock();
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0,0, visCvRaw->width,visCvRaw->height,
                    GL_BGR, GL_UNSIGNED_BYTE, 
                    //visCvAdapt->imageData
                    visCvRaw->imageData
                    );
	//free the image data so it can be updated again.
	//CamSource->unlock();

	//ft->render_redirect( tex );
	//retrieve our features into a scene,s
    Scene s;
    ft->getScene(tex,s);

	//save first scene as reference.
    if ( refScene == NULL ) refScene = new Scene(s);
    else if ( refreshRefScene == true ) {
        refScene = new Scene(s);
        refreshRefScene = false;
        fprintf(stderr, "----------scene-refresh------------\n");
    }
    PCT matchedPCT;
    Matches bestM = s.MatchTo( refScene, visCvRaw->width, visCvRaw->height, matchedPCT );


    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, ft->tex[viewbuf] );
    reshape(width, height);

    cgGLEnableProfile(cgProfile);
    cgGLBindProgram(basicProgram);

    glBegin(GL_QUADS);

    glTexCoord2f(0, visCvRaw->height );
    glVertex3f(0.0, 1.0, d);

    glTexCoord2f(0, 0);
    glVertex3f(0.0, 0.0,d );

    glTexCoord2f(visCvRaw->width, 0);
    glVertex3f(1.0, 0.0, d );

    glTexCoord2f(visCvRaw->width, visCvRaw->height );
    glVertex3f(1.0,1.0, d);
    glEnd();

    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    cgGLDisableProfile(cgProfile);

    drawFeatures( &s );

    drawMatches( bestM ) ;

    glutSwapBuffers();
    showstats();
    

}

int main(int argc, char *argv[] )  {

	//start the camera capture
    if (argv[1]==NULL)
        /* connect to the camera */
        CamSource.connect();
    else
        /* load a video */
        CamSource.connect(0, argv[1]);
	//must call right after connecting to camera
	init();


    int CaptureWidth = visCvRaw->width, CaptureHeight = visCvRaw->height;
    int DisplayWidth = visCvRaw->width, DisplayHeight = visCvRaw->height;

    width = DisplayWidth;
    height = DisplayHeight;

    glutInit( &argc, argv );
    
    glutInitWindowSize(visCvRaw->width, visCvRaw->height);
    glutInitWindowPosition(800, 480); 	
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowSize(width, height );
    glutCreateWindow(argv[0]);

    ft = new featureTrack(visCvRaw->width,visCvRaw->height );

	// make a texture
    glGenTextures(1, &tex);              // texture
    glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, visCvRaw->width,visCvRaw->height, 0, GL_BGR, GL_UNSIGNED_BYTE, visCvRaw->imageData );

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

void drawFeatures( Scene *currentScene )
{
    int w = ft->width();
    int h = ft->height();
    cerr<<"Drawing "<<currentScene->features.size()<<" features\n";
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(CG_PROFILE_FP40);
    glDisable(CG_PROFILE_FP30);
    glBegin(GL_LINES);
    for ( vector<Feature>::iterator it = currentScene->features.begin() ;
            it != currentScene->features.end();
            it++ )
    {
        glColor3f( 0.0, 0.0, 1.0 );
        glVertex3f( it->x()/(float)w ,
                    it->y()/(float)h , -1.0 );
        glVertex3f( it->x()/(float)w+ it->dx*2.0 ,
                    it->y()/(float)h +it->dy*2.0 , -1.0 );


        glColor3f( 0.0, 1.0, 0.0 );
        Coords  corner;
        Coords  tmp;
        
        float xoffset = -8.0;
        float yoffset = -8.0;
        tmp.set( xoffset, yoffset );
        corner.first = cosf( it->orientation) * tmp.first -
                       sinf( it->orientation) * tmp.second ;
        corner.second = sinf( it->orientation) * tmp.first +
                        cosf( it->orientation) * tmp.second ;
        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );
                    
        xoffset = -8.0;
        yoffset = +8.0;
        tmp.set( xoffset, yoffset );
        corner.first = cosf( it->orientation) * tmp.first -
                       sinf( it->orientation) * tmp.second ;
        corner.second = sinf( it->orientation) * tmp.first +
                        cosf( it->orientation) * tmp.second ;
        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );

        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );

        xoffset = +8.0;
        yoffset = +8.0;
        tmp.set( xoffset, yoffset );
        corner.first = cosf( it->orientation) * tmp.first -
                       sinf( it->orientation) * tmp.second ;
        corner.second = sinf( it->orientation) * tmp.first +
                        cosf( it->orientation) * tmp.second ;
        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );

        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );
                    
        xoffset = +8.0;
        yoffset = -8.0;
        tmp.set( xoffset, yoffset );
        corner.first = cosf( it->orientation) * tmp.first -
                       sinf( it->orientation) * tmp.second ;
        corner.second = sinf( it->orientation) * tmp.first +
                        cosf( it->orientation) * tmp.second ;
        glVertex3f( (it->x() + corner.first )/(float)w,
                    (it->y() + corner.second )/(float)h ,
                    -1.0 );
    }
    glEnd();

}

void drawMatches( Matches &bestM )
{
    int w = ft->width();
    int h = ft->height();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(CG_PROFILE_FP40);
    glDisable(CG_PROFILE_FP30);
    glLineWidth(1.0);
    glColor3f( 1.0, 1.0, 0.0);
	//if( bestM.size() >= 4  && bestP.norm() < 1.0) {
    if ( bestM.size() >= 4  ) {
        glBegin(GL_LINES);
        for ( Matches::iterator it=bestM.begin(); it != bestM.end(); it++ ) {
            glVertex3f( it->first->x()/(float)w ,
                        it->first->y()/(float)h , -1.0);
            glVertex3f( it->second->x()/(float)w,
                        it->second->y()/(float)h, -1.0 );

        }
        glEnd();
    }
}

void init()
{
	CamSource.GrabCvImage();
	visCvDebug = NULL; //cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 3);
	visCvAdapt = cvCreateImage(cvSize(visCvRaw->width,visCvRaw->height), IPL_DEPTH_8U, 1);
    visCvThresh = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
    visCvPath = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);
    visCvAdaptSmall = cvCreateImage(cvSize(visCvRaw->width/2,visCvRaw->height/2), IPL_DEPTH_8U, 1);    	
    cvNamedWindow("display",0);
    trackbarVal=1;
    int numberOfViews = 10; // important!!!
	cvCreateTrackbar("bar","display",&trackbarVal,numberOfViews,trackbarHandler);
    cvResizeWindow( "display", visCvRaw->width, visCvRaw->height );
    cvMoveWindow( "display", 10, 10 ); // position on screen
    cvMoveWindow( "roi", visCvRaw->width/2-20, 60+visCvRaw->height );

}
void trackbarHandler(int pos)
{
    printf("pos = %d \n", pos);
}
void dovision()
{
	visCvDebug = cvCloneImage( visCvRaw );
	vp.visAdaptiveProcessing();
	
    //cvSmooth(visCvRaw, visCvRaw);
    cvErode(visCvRaw,visCvRaw,NULL,2);
    cvSmooth(visCvRaw, visCvRaw);
    //int thresh = 20;
    //cvThreshold(visCvRaw,visCvRaw, thresh, 255, CV_THRESH_BINARY );
	
	vp.ConvertAllImageViews(trackbarVal); 
	cvReleaseImage(&visCvDebug);
}




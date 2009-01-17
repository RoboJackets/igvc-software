/* COMPILE:

g++ featureExample.cc  -I../include -L../ -I/usr/include/cc++ -I/usr/include/cc++2/ -ldc1394_control -lraw1394  -lstdc++ -lccext2 -lccgnu2 -lxml -lopenvidia -lpthread -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lImlib -I~/SDK/LIBS/inc -lCgGL

  */

#include <FL/Fl_Gl_Window.H>
#include <GL/gl.h>
#include <GL/glext.h>
#include <Cg/cgGL.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <vector>
#include <openvidia/openvidia32.h>
#include <GL/glx.h>
#include <GL/glxext.h>
#include <Imlib2.h>
#include "InterfaceWindow.h"

int DisplayWidth = 320, DisplayHeight = 240;
CamParams CameraParameters(
    723.99739/2.0 , //f1 (fx)
    722.97928/2.0 , //f2 (fy)
    326.06551/2.0 , //ox
    255.89249/2.0 , //oy
    -0.33246 , //k1
    0.01839 , //k2
    -0.00020 , //k3
    0.00022  );

using namespace std;

GLuint tex, overlayTex ;

bool use_bgr= false;
LBuffer *CamSource=NULL;

featureTrack *ft;
int width, height;
CGprogram basicProgram, overlayProgram;
CGprofile cgProfile;
CGcontext cgContext;
bool refreshRefScene = false;
bool saveRefScene = false;

//Vars for image overlay.
Imlib_Image Im;


void drawFeatures( Scene *currentScene );
void drawMatches( Matches &bestM ) ;
// callbacks for opengl
void render_redirect() ;
void keyboard (unsigned char key, int x, int y);
bool CreateOverlayTexture( char *matchedString );

CGprogram load_cgprogram(CGprofile prof, char *name) {
    fprintf(stderr, "loading %s\n", name);
    return cgCreateProgramFromFile( cgContext, CG_SOURCE,
                                    name, prof, "FragmentProgram", 0);
}

///FLTK windowing functions:

Fl_Window *window;
UserInterface *InterfaceWindow;

void init_gl() ;

void IdleCallback(void* pData)
{
    window->redraw();
}

class MyWindow : public Fl_Gl_Window {
    void draw();
    int handle(int);

public:
    MyWindow(int X, int Y, int W, int H, const char *L)
            : Fl_Gl_Window(X, Y, W, H, L) {
        Fl::add_idle(IdleCallback, this);
        this->mode(FL_RGB8 | FL_DEPTH |FL_ALPHA | FL_DOUBLE | FL_STENCIL);
    }
    void ddraw() {
        draw();
    }
};

void MyWindow::draw() {
    static bool firsttime = true ;
    if ( firsttime ) {
        init_gl();
        firsttime = false;
    }
    render_redirect();
}

int MyWindow::handle(int x ) {
    switch (x) {
    case FL_KEYDOWN :
        keyboard( *(Fl::event_text()) , 0, 0);
        break;
    default :
        break;
    }
}


float ls, rs, ts, bs , dist ;

void reshape(int w, int h)
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(0.0, 1.0,  0.0, 1.0,   1.0,   100.0);
//  glFrustum(  (ls)/CameraParameters.f1(), rs/CameraParameters.f1() ,
//              (bs)/CameraParameters.f1(), ts/CameraParameters.f1(),
//              1.0, CameraParameters.f1()+1000.0f );



    gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  -1.0,   0.0, 1.0, 0.0);
    //gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  -dist,   0.0, 1.0, 0.0);

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
        //fprintf(stderr, "fps = %d,  %f msecs\n",fps,1.0/((float)fps)*1000);
        char tmp[5];
        sprintf(tmp, "%d", fps);
        InterfaceWindow->FPSOutput->value( tmp );

    } else {
        fpscounter++;
    }
}


void myIdle() {
    //glutPostRedisplay();
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


Scene *refScene = NULL;

void render_redirect() {

    Scene *newScene ;

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
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0,0, CamSource->width(),
                    CamSource->height(), GL_RGB, GL_UNSIGNED_BYTE,CamSource->ptr());
    //free the image data so it can be updated again.
    CamSource->unlock();

    //ft->render_redirect( tex );
    //retrieve our features into a scene,s
    Scene scn;
    ft->getScene(tex, scn);
    Scene *s = &scn;

    //save first scene as reference.
    //if( refScene == NULL ) refScene = s;


    if ( refScene == NULL ) refScene = new Scene(*s);
    if ( refreshRefScene == true ) {
        refScene = new Scene(*s);
        refreshRefScene = false;
    }
    if ( saveRefScene == true ) {
        s->saveToDisk("scene");
        saveRefScene = false;
    }


    // iterate through loaded scenes.


    Matches bestM;

    PCT bestP;
    static PCT lastP;

    int matchedIndex = 0;
    char matchedName[100];

    // iterate thru the list of loaded scenes, and look for a best
    // match.
    for ( int i =1 ; i<=InterfaceWindow->LoadedScenesBrowser->size(); i++ )
    {
        PCT someP;
        assert( (Scene *)InterfaceWindow->LoadedScenesBrowser->data(i) != NULL );
        Matches matchMetric = s->MatchTo((Scene *)InterfaceWindow->LoadedScenesBrowser->data(i), CamSource->width(),CamSource->height(), someP);

        if ( matchMetric.size() > bestM.size() )
        {
            bestM = matchMetric;
            matchedIndex = i;
            bestP = someP;
        }
    }
    //examine the best match found.  If enough corners are matched, use it.
    if ( bestM.size() > 4 ) {
        char tmp[200];
        strcpy( tmp, InterfaceWindow->LoadedScenesBrowser->text( matchedIndex ) );
        strcpy( matchedName, strtok( tmp, (const char *)("\t" ) ) );

        int iter = 1;



        //iteratively re-train the PCT to include possibly missed matches from the RANSAC PCT.
        // The ransac PCT was a rough/random estimate. the SVD version is a better behaved and
        // minimzes misalignment error.  So, after we generate a SVD version, check the matches
        // again to see if we shuld include more.  Do this until no new matches are made.
        while (true)
        {
            bestP = PCT( bestM, CamSource->width(), CamSource->height(), METHOD_SVD);

            // Method A : Utilize feature positions.   Features less likely to alias to one another since some position is known.
            Matches newMatches = findMatchesByPosition(  (Scene &)(*((Scene *)InterfaceWindow->LoadedScenesBrowser->data(matchedIndex))), (Scene &)*s, 2, bestP,CamSource->width(), CamSource->height() );
            /*
                  // Method B : re-iterate to retrain more features which may have been missed outside the ransac.
                  Matches m = findMatches( (Scene &)(*((Scene *)InterfaceWindow->LoadedScenesBrowser->data(matchedIndex))), (Scene &)*s, 1.0, 0.8);
                  // because we have a better estimate of motion, we could decrease the search radius tolerance.
                  Matches newMatches = s->getSupport( bestP, m, 6, CamSource->width(), CamSource->height() );
            */

            if ( (bestM.size() == newMatches.size()) || (iter > 4) ) {
                //for( Matches::iterator i = newMatches.begin() ; i != newMatches.end() ; i++ ) { i->print(); }
                bestM = newMatches;
                break;
            }
            if ( newMatches.size()==0 ) {
                strcpy( matchedName, "" ) ;
                cerr<<"!!!"<<endl;
                break ;
            }
            iter++;
            break; // remove this line to actually iterate. not necessaryily...necessary...
        }
        //cerr<<"Finished after "<<iter<<" iterations"<<endl;
        //Matches PositionMatches = findMatchesByPosition(  (Scene &)(*((Scene *)InterfaceWindow->LoadedScenesBrowser->data(matchedIndex))), (Scene &)*s, 2, bestP,CamSource->width(), CamSource->height() );

//  cerr<<"SIFT: "<<bestM.size()<<"  Positional: "<<PositionMatches.size()<<endl;

    }
    //if not enough matches, nothing has been found.
    else {
        strcpy( matchedName, "" );
    }
    if ( bestP.FirstSingVal() < 4.0 ) {
        strcpy( matchedName, "" );
    }
    InterfaceWindow->MatchedToOutput->value( matchedName );


    /* for the next version.  need to implement trim operations.

      if( InterfaceWindow->AddFeaturesButton->value() && strcmp(matchedName,"") ) {
        InterfaceWindow->AddFeaturesButton->value(0);
        PCT ptmp = bestP;
        !ptmp;
        ((Scene *)InterfaceWindow->LoadedScenesBrowser->data(matchedIndex))->AddFeaturesToScene( s, &bestM, ptmp, CamSource->width(), CamSource->height() );
      }
    */
    //output status to interface window.
    char output[25];
    sprintf(output, "%d", bestM.size() );
    InterfaceWindow->FeaturesMatchedOutput->value(output);
    sprintf(output, "%d",s->features.size());
    InterfaceWindow->FeaturesFoundOutput->value(output);

    // draw the image texture.
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, ft->tex[viewbuf] );
    reshape(DisplayWidth, DisplayHeight);

    cgGLEnableProfile(cgProfile);
    cgGLBindProgram(basicProgram);

    glBegin(GL_QUADS);

    glTexCoord2f(0, CamSource->height() );
    glVertex3f(0.0, 1.0, d);
    //glVertex3f(ls, ts, -dist);

    glTexCoord2f(0, 0);
    glVertex3f(0.0, 0.0,d );
    //glVertex3f(ls, bs,-dist );

    glTexCoord2f(CamSource->width(), 0);
    glVertex3f(1.0, 0.0, d );
    //glVertex3f(rs, bs, -dist );

    glTexCoord2f(CamSource->width(), CamSource->height() );
    glVertex3f(1.0,1.0, d);
    //glVertex3f(rs,ts, -dist);

    glEnd();

    glDisable(GL_TEXTURE_RECTANGLE_NV);

    // draw an associated overlay for the image found.
    // create the texture. then display it if anything is to be shown
    if ( CreateOverlayTexture( matchedName ) ) {
        // draw overlay texture.
        glEnable(GL_TEXTURE_RECTANGLE_NV);

        glClear(GL_DEPTH_BUFFER_BIT);
        glBindTexture(GL_TEXTURE_RECTANGLE_NV, overlayTex);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4f(0.0,0.0,1.0,1.0);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();


        // dejitter a bit.  Only update motion if it moves a certain amount,
        // otherwise use the old value.
        PCT P;
        //cerr<<(bestP-lastP).norm() <<endl;

        if ( (bestP-lastP).norm() > 0.0001 )
        {
            P = bestP ;
            lastP = bestP;
        }
        else
        {
            P = lastP ;
        }

        float denom = P.c1() * 0.5 + P.c2() * 0.5 + 1.0 ;
        float topY  = P.a11()*0.5 + P.a12()*0.5 + P.b1();
        float topX  = P.a21()*0.5 + P.a22()*0.5 + P.b2();

        glTranslatef( topX/denom-0.5, topY/denom-0.5, 0.0);

        glBegin(GL_QUADS);
        glTexCoord2f(0, imlib_image_get_height() );
        glVertex3f(0.0, 0.0, d );
        //glVertex3f(ls, bs, -dist);

        glTexCoord2f(0, 0);
        glVertex3f(0.0, 1.0, d);
        //glVertex3f(ls, ts, -dist);

        glTexCoord2f(imlib_image_get_width(), 0);
        glVertex3f(1.0, 1.0, d);
        //glVertex3f(rs, ts, -dist);

        glTexCoord2f(imlib_image_get_width(), imlib_image_get_height() );
        glVertex3f(1.0,0.0, d);
        //glVertex3f(rs,bs , -dist);
        glEnd();
        glPopMatrix();


        glDisable(GL_BLEND);
        glDisable(GL_TEXTURE_RECTANGLE_NV);
    }

    cgGLDisableProfile(cgProfile);


    if ( InterfaceWindow->ShowFeatures->value() )
    {
        drawFeatures( s );
        drawMatches( bestM ) ;
    }

    //delete(s);
    //glutSwapBuffers();
    showstats();
}

void load_image ( char *fname )
{

    fprintf(stderr,"Opening %s\n", fname );

    Im  = imlib_load_image(fname );

    if ( Im == NULL ) {
        fprintf(stderr," Could not load the image\n");
    }

}


void init_gl() {
    ft = new featureTrack(CamSource->width(), CamSource->height()  );
    ft->setCamParams( CameraParameters );

    ls = (((double)320.0)-(320.0-CameraParameters.ox()));
    rs = -(320.0-CameraParameters.ox());
    bs = (((double)240.0)-CameraParameters.oy());
    ts =  -CameraParameters.oy();
    dist = CameraParameters.f1();

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
    if ( use_bgr )
    {
        basicProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-BGR.cg");
    }
    else
    {
        basicProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-basic.cg");
    }
    cgGLLoadProgram( basicProgram );
    overlayProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-overlay.cg");
    cgGLLoadProgram( overlayProgram );


    //make a texture for the overlay
    // load_image("testoverlay.png");
    // imlib_context_set_image(Im);
    glGenTextures(1, &overlayTex);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, overlayTex);
    glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
    // if( imlib_image_has_alpha() ) cerr<<"alpha detected"<<endl;
    /*
       glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, imlib_image_get_width() ,
                    imlib_image_get_height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                    imlib_image_get_data() );
    */

}

int main(int argc, char *argv[] )  {

    // the initial width/height settings here are what is desired.
    // Cam source may use different settings based on what the camera can do ,
    // the the CamSource object should be asked after it is initalized what
    //  the actual dimensions of capture are after and you can use those
    // as needed after.
    int CaptureWidth = 320, CaptureHeight = 240;

    // allow user to pick device type.
    if ( argc > 1 ) {
        // check if the last argument is --v4l1 or --1394
        if ( strcmp( argv[ argc-1 ], "--V4L1" )==0||
                strcmp( argv[ argc-1 ], "--v4l1" )==0 ) {
            cerr<<"Request V4L1"<<endl;
            CamSource = new V4L1(CaptureWidth, CaptureHeight);
        }
        if ( strcmp( argv[ argc-1 ], "--1394" )==0 ) {
            cerr<<"Request 1394"<<endl;
            CamSource = new Dc1394(
                CaptureWidth,
                CaptureHeight
            );
        }
        //remove the last argument from argv so parsing can occur as normal
        // in other functions (as though --v4l1 was never passed )
        argv[argc-1] = NULL;
        argc--;
    }

    if ( CamSource == NULL )
    {
        cerr<<"You MUST specify --V4L1 or --1394 as the LAST argument to pick cam.";
        cerr<<endl;
        exit(0);
    }

    if ( argc > 1 ) {
        if ( strcmp( argv[argc-1], "--bgr" ) == 0 ) {
            cerr<<"Requesting BGR color conversion."<<endl;
            use_bgr = true;
            //remove the last argument from argv so parsing can occur as normal
            // in other functions (as though --v4l1 was never passed )
            argv[argc-1] = NULL;
            argc--;
        }
    }


    width = CamSource->width();
    height = CamSource->height();

    DisplayWidth = width;
    DisplayHeight = height;

    window = new MyWindow(0,0, DisplayWidth, DisplayHeight,"FLfeature" );
    //window = new MyWindow(0,0, 800, 600,"FLfeature" );
    Fl::gl_visual( (FL_RGB8 | FL_DEPTH |FL_ALPHA | FL_DOUBLE) );

    window->end();
    window->show(argc, argv);

    InterfaceWindow = new UserInterface(window);
    //InterfaceWindow->end();
    InterfaceWindow->show(argc,argv);

    return Fl::run();
}

void drawFeatures( Scene *currentScene )
{
    int w = ft->width();
    int h = ft->height();
    //cerr<<"Drawing "<<currentScene->features.size()<<" features\n";
    glDisable(GL_TEXTURE_RECTANGLE_NV);
    glDisable(CG_PROFILE_FP40);
    glDisable(CG_PROFILE_FP30);
    glBegin(GL_LINES);
    for ( vector<Feature>::iterator it = currentScene->features.begin() ;
            it != currentScene->features.end();
            it++ ) {
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
    glDisable(GL_TEXTURE_RECTANGLE_NV);
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

bool CreateOverlayTexture( char *matchedString )
{
    static char lastMatch[200];

    // if nothing was found
    if ( !strcmp("", matchedString) )
    {
        strcpy( lastMatch, "" );
        return false ;
    }

    // if we found a new scene, thats not the old one, load in the overlay
    if ( strcmp( matchedString, lastMatch ) )
    {
        //make a texture for the overlay
        char overlayName[240];
        strcpy( lastMatch, matchedString );

        sprintf( overlayName, "%s-overlay.png", matchedString );
        if ( Im != NULL )  {
            imlib_context_set_image(Im);
            imlib_free_image() ;
            Im = NULL ;
        }
        load_image(overlayName);

        if ( Im == NULL )
        {
            return false ;
        }
        imlib_context_set_image(Im);
        glDeleteTextures(1, &overlayTex );
        glGenTextures(1, &overlayTex);
        glBindTexture(GL_TEXTURE_RECTANGLE_NV, overlayTex);
        glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, imlib_image_get_width() ,
                     imlib_image_get_height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     imlib_image_get_data() );
    }
    else {
        assert( !strcmp( matchedString, lastMatch ) );
    }
    if ( Im != NULL )
    {
        return true;
    }
    else
    {
        return false;
    }
}

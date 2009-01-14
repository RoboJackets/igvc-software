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
#include <openvidia/libopenvidia.h>
#include <../src/V4L1.h>


using namespace std;
ImlibImage *Im;

GLuint tex ;

featureTrack *ft;
int width, height;
CGprogram basicProgram;
CGprofile cgProfile;
CGcontext cgContext;
bool refreshRefScene = false;

void drawFeatures( Scene *currentScene );
void drawMatches( Matches &bestM ) ;
PCT RANSAC( Matches &m ) ;
Matches getSupport( PCT &P, Matches &m, float radius );

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
      case 'r' :  //request that new refernce scene be saved - will use the imediate next frame.
          refreshRefScene = true;
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
  static Scene *refScene = NULL;

  Scene *newScene ;

  float d =  -1.0;

  glClearColor(0.0, 0.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glEnable(GL_TEXTURE_RECTANGLE_NV);
  glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex) ;

  // This is how you'd load the next image in the sequence
  // fname would be the next file name
  // and i think you may need to put the imlib_id variable
  //  in global scope (its in the load_image function right now ).
  //  Im  = Imlib_load_image(imlib_id, fname );

  glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0,0, Im->rgb_width, Im->rgb_height,
                GL_RGB, GL_UNSIGNED_BYTE,Im->rgb_data);
  //free the image data so no memory leaks

  //Imlib_destroy_image(....) see
  // http://wolfpack.twu.net/docs/Imlib/tutorial.html
 

  //ft->render_redirect( tex );
  //retrieve our features into a scene,s
  Scene *s = ft->getScene(tex);

  //save first scene as reference.
  if( refScene == NULL ) refScene = s;
  else if ( refreshRefScene == true ) {
     refScene = s;
     refreshRefScene = false;
  }

  //go through the scenes, find al the matches.
  Matches m = findMatches( *s, *refScene, 1.0, 0.8 );

  // RANSAC
  int bestSupport = 0;
  Matches bestM;
  PCT bestP; 

  for( int i=0; i<100; i++ ) {
    //create a PCT from 4 random matches
    PCT P = RANSAC( m );
    //discard bizarre PCTs
    if( P.norm() > 2.0 ) continue;
    // find out how well supported this PCT is
    Matches supportM = getSupport( P, m, 6 );
    // record the best supported PCT.
    if( supportM.size() > bestSupport ) {
        bestSupport = supportM.size();
        bestP = P;
        bestM = supportM;
        //P.print();
    }
  }
  cerr<<" Best support "<<bestSupport<<" : "<<bestM.size() <<" : " ;

  


  glBindTexture(GL_TEXTURE_RECTANGLE_NV, ft->tex[viewbuf] );
  reshape(width, height);

  cgGLEnableProfile(cgProfile);
  cgGLBindProgram(basicProgram);

  glBegin(GL_QUADS); 

    glTexCoord2f(0, Im->rgb_height );
    glVertex3f(0.0, 1.0, d);

    glTexCoord2f(0, 0);
    glVertex3f(0.0, 0.0,d );

    glTexCoord2f(Im->rgb_width, 0);
    glVertex3f(1.0, 0.0, d );

    glTexCoord2f(Im->rgb_width, Im->rgb_height );
    glVertex3f(1.0,1.0, d);
  glEnd();

  glDisable(GL_TEXTURE_RECTANGLE_NV);
  cgGLDisableProfile(cgProfile);

  drawFeatures( s );

  drawMatches( bestM ) ;

  // toss the scene away after we're done to avoid memory leak
  // except dont delete the first refernece scene.
  if( refScene != s ) delete(s);
  glutSwapBuffers();
  showstats();
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
}
  
 
int main(int argc, char *argv[] )  {

   int CaptureWidth = 320, CaptureHeight = 240;
   int DisplayWidth = 800, DisplayHeight = 600;


   if( argc != 2 ) {
     cerr<<"Give 1 filename as an argument\n";
     exit(0);
   } 

   load_image(argv[1]);

   CaptureWidth = Im->rgb_width;
   CaptureHeight = Im->rgb_height;
   DisplayWidth = CaptureWidth;
   DisplayHeight = CaptureHeight;

   width = Im->rgb_width;
   height = Im->rgb_height;

   cerr<<"size "<<width<<"x"<<height<<endl;

   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH |GLUT_STENCIL);
   glutInitWindowSize(width, height );
   glutCreateWindow(argv[0]);


   ft = new featureTrack(Im->rgb_width, Im->rgb_height );

   // make a texture
   glGenTextures(1, &tex);              // texture 
   glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
   glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
   glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, Im->rgb_width , 
                Im->rgb_height, 0, GL_RGB, GL_UNSIGNED_BYTE,NULL );


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
    glDisable(GL_TEXTURE_RECTANGLE_NV);
    glDisable(CG_PROFILE_FP40);
    glDisable(CG_PROFILE_FP30);
    glBegin(GL_LINES);
    for( vector<Feature>::iterator it = currentScene->features.begin() ;
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

PCT RANSAC( Matches &m )
{  
  //create a Matches object to hold our selected correspondences.
  Matches ransacCorrs ;
  if( m.size() < 4 ) {
    return PCT() ;
  }
  //choose 4 correspondences to define a PCT. 
  while( ransacCorrs.size() < 4 ) {

    //create a random index into the matches vector to choos a random match
    int idx = (int)(floorf((float)rand()/((float)RAND_MAX)*(float)m.size() ));

    //make sure we dont re-use the same data point.
    //todo: if a collision, then use a circular linear search instead of reprobe
    if( find( ransacCorrs.begin(), 
              ransacCorrs.end(), m[idx] ) == ransacCorrs.end() ) {
      ransacCorrs.push_back(m[idx]);
    }
  }
  PCT P( ransacCorrs, height, width );
/*
  //print out corr2p
  Matches::iterator it;
  it = ransacCorrs.begin();
  cerr<<endl<<height<<" "<<width<<" ";
  while( it != ransacCorrs.end() ) {
    cerr<<it->first->y()<<" "<<it->first->x()<<" ";
    cerr<<it->second->y()<<" "<<it->second->x()<<" ";
    it++;
  }
  cerr<<endl;
  P.print();
  cerr<<endl;
  ~P;
  P.print();
  cerr<<endl;
*/
  return P;
}


///given an array of matches, m, how many support the hypothesized 
/// PCT, P.
Matches getSupport( PCT &P, Matches &m, float radius )
{
   Matches supportedMatches ;
   float radiusSq = radius*radius;
   int support =0;
   Coords n;
   Matches::iterator it;

   it = m.begin();
   while( it != m.end() ) {


     if( distSq( (n = P.project( *(it->first), width, height )),
                 *(it->second) )
         < radiusSq ) {
       support++;
/*
     cerr<<"Comparing "<< it->first->x() <<", "<< it->first->y() ;
     cerr<<" to "<< it->second->x() << ", " << it->second->y() << endl;
     n = P.project( *(it->first), width, height ) ;
     cerr<<"With "; P.print();
     cerr<<endl<< it->first->x() <<", "<< it->first->y()<<" projects to ";
     cerr<<n.x()<<" "<<n.y()<<endl;
     cerr<<"distance of "<<distSq( (n = P.project( *(it->first), width, height )), *(it->second) );
*/
       supportedMatches.push_back( *it );
     }
     it++;
   }
   return supportedMatches ;
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
  if( bestM.size() >= 4  ) {
  glBegin(GL_LINES);
    for( Matches::iterator it=bestM.begin(); it != bestM.end(); it++ ) {
      glVertex3f( it->first->x()/(float)w ,
                  it->first->y()/(float)h , -1.0);
      glVertex3f( it->second->x()/(float)w,
                  it->second->y()/(float)h, -1.0 );

    }
  glEnd();
  }
}

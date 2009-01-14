#ifndef __OPENVIDIA32_H
#define __OPENVIDIA32_H

#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <math.h>

#ifdef WIN32
#include <GL/glew.h>
#endif

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>

#ifdef WIN32
#define GLEW_STATIC 1
#endif

#include <GL/glut.h>
#include <Cg/cgGL.h>

using namespace std;

#ifdef WIN32
# ifdef DLL_EXPORT
#   define DECLSPEC __declspec(dllexport)
#   define EXPIMP_TEMPLATE
# else
#  define DECLSPEC __declspec(dllimport)
#  define EXPIMP_TEMPLATE extern
#endif

  EXPIMP_TEMPLATE template class DECLSPEC std::vector<float>;
  EXPIMP_TEMPLATE template class DECLSPEC std::pair<float,float>;

#else
# define DECLSPEC
# define EXPIMP_TEMPLATE
#endif


class DECLSPEC Coords : public pair<float,float>{
  public : 
    Coords( float a=0, float b=0 ) { first = a; second = b; }
    //look like the vgl libs version for future association
    float x() { return first; }
    float y() { return second; }
    void set(float in1, float in2 ) {
      first = in1; 
      second = in2;
    }
    void print() {
      cerr<<" "<<x()<<" "<<y()<<" ";
    }
};

///\brief The feature class holds the information about a feature found in an image.
///
/// The feature class holds the information about a feature found in an image.
/// It inherits from the Coords class, so has an x() and y() denoting its position
/// in the image.    See the fvutil.h file for some utility functions that operate
/// on features.  Currently these are created by the featureTrack object.
class DECLSPEC Feature : public Coords {
  public:
	float descArray[128];
//win32 and linux use descArray.
//linux g++4.0 needs the descriptor though
//because if no push back call is made , -o3 optimizes the 
//assignement out and results in 0 (see featureTracker.getScene() )
#ifndef WIN32
   vector<float> descriptor;        ///< 128 element feature descriptor (SIFT "key")
                                     ///  It can be considered to "match" another
                                     ///  another feature by calculating typical euclidean distance.
#endif
    float orientation;               ///< Orientation of the feature = atan2f( dy, dx )
    float dx;                        ///< X derivative in a 16x16 region around the feature centre.
    float dy;                        ///< Y derivative in a 16x16 region around the feature centre. 
    float magnitude;                 ///< Magnitude of the gradient = sqrt( dx^2 + dy^2 ) 

    Feature() {
      //descriptor.reserve(128*sizeof(float));
    }
    /// returns the norm of the feature descriptor.
    double norm() {                  
      double n=0.0;
      for( int i=0 ;i<128; i++ ) { 
        //n += descriptor[i]*descriptor[i];
		n+=descArray[i]*descArray[i];
      } 
      return sqrt(n);
    }
    void print() {
      cerr<<"[";
      for( int i = 0 ; i < 4 ; i ++ ) {
        cerr<<descArray[i]<<" ";
      } 
      cerr<<"]"<<endl;
    }

};

//typedef pair<Feature *, Feature *> Match ;
#ifdef WIN32
EXPIMP_TEMPLATE template class DECLSPEC std::pair<Feature *, Feature *>;
#endif
/// a Match is an STL pair of pointers to two features.
class DECLSPEC Match : public pair<Feature *, Feature *> {
  public:
	  Match() {};
    Match( Feature *a, Feature *b ) : pair<Feature *,Feature*>( a, b ) {} 

    /// Returns true if the given match, op,  is the same set of pointers.
    bool operator==(Match op) {    
      return ( ( first==op.first )&&( second==op.second ) );
    }
    void print() {
      cerr<<first->x()<<","<<first->y()<<" -> "<<second->x()<<","<<second->y()<<endl;
    }
};

/// Matches is a STL vector of Match classes.
typedef vector<Match> Matches ;

#ifdef WIN32
EXPIMP_TEMPLATE template class DECLSPEC std::vector<Match>;
#endif


#define METHOD_EXACT 0
#define METHOD_SVD   1

class DECLSPEC PCT : public vector<float>{
  private :
    float _FirstSingVal;
    void set( double a11, double a12, double b1, 
         double a21, double a22, double b2, 
         double c1,  double c2 ) ;
  public :
    PCT( ); 
    PCT( Matches &, int, int, int );
    PCT( float a11in, float a12in, float b1in, float a21in, 
         float a22in, float b2in,  float c1in, float c2in );
    
    float a11() { return (*this)[0]; } 
    float a12() { return (*this)[1]; } 
    float b1()  { return (*this)[2]; } 

    float a21() { return (*this)[3]; } 
    float a22() { return (*this)[4]; } 
    float b2()  { return (*this)[5]; } 

    float c1() { return (*this)[6]; } 
    float c2() { return (*this)[7]; } 

    void print() {
      PCT::iterator it = begin();
      while( it != end() ) {
        cerr<<*it++<<" ";
      }
      cerr<<endl;
    }

    void operator!();
    Coords project( Coords &p, int W, int H ) ;

    float norm() {
     return (a11()-1.0)*(a11()-1.0) + a12()*a12() + a21()*a21() +
            (a22()-1.0)*(a22()-1.0) + b1()*b1() + b2()*b2() +
            c1()*c1() + c2()*c2() ;
    }

    PCT operator-(PCT pct2);
    double FirstSingVal();
    
};



class  DECLSPEC class1 : public std::vector<float> { 
//private :
//   std::vector  <float>XX;
public: int getX();
};
EXPIMP_TEMPLATE template class DECLSPEC std::vector<Feature>;

class DECLSPEC Scene {
  public:
    vector<Feature> features;

    /// Saves a scene to a disk.  
    /// The format is ASCII text integer of the number of 
    /// features in the scene, followed by
    /// the feature vectors, 1 per line as:
    /// X Y Orientation dx dy magnitude [ 128 element descriptor ]
    /// (elements separated by spaces)

  
    void saveToDisk(const char *filename) ;

    /// Default constructor creates empty scene (empty vector of features )
    Scene() ;
	Scene(int x);

    /// Constructor is given a filename, which is loaded.
    Scene(const char *filename) ;
    Matches MatchTo( Scene *s , int w, int h, PCT &bestP) ;
    Matches getSupport( PCT &P, Matches &m, float radius, int w, int h );

    int  TrimOutMatches( Matches *m);
    void AddFeaturesToScene( Scene *toAdd, Matches *m, PCT P, int w, int h );

  private :
    PCT RANSAC( Matches &m, int w, int h );
 

};



class DECLSPEC FBO_Filter {

protected :
    CGprogram cgProgram;
    CGprofile cgProfile;
    CGcontext cgContext;

    GLuint fb, oTex;
    int tWidth; int tHeight;
    int previousViewportDims[4] ;
    float depth;

    virtual CGprogram load_cgprogram(CGprofile prof, char *name, char **args) ;

    void renderBegin() ;
    //return the rendering state to what it was before.
    void renderEnd() ;

    virtual void drawQuadTex();

    virtual void drawQuadFBT();


public :
    
    FBO_Filter( CGprofile cgp,      ///< Desired profile.  Typically 
                                    ///  CG_PROFILE_FP30 or CG_PROFILE_FP40
                char *name,         ///< filename of the Cg program.  Note that
                                    /// the entry point function should be named
                                    /// "FragmentProgram" in the Cg 
                GLuint outputTex,   ///< the open GL texture object 
                                    /// to which the results will go 
                 int W,             ///< width of the output texture, 
                 int H,             ///< height of the output texture

                 /// (Optional) arguments to the cgc compiler. Useful for
                 /// example to specify compile time #defines for example
                 char **args  = NULL );
   


  
    ///TODO figure out readback formats from target texture information.
    GLuint apply( GLuint iTex,    ///< OpenGL texture object to use as input
                  bool FBOtex,     ///< set to true if the input texture is the result of a previous 
                                  ///  FBO_filter.  False if not.
                  GLvoid *rb_buf = NULL, 
                  GLenum rb_fmt = GL_RGBA, 
                  GLenum rb_type = GL_FLOAT );
};


class DECLSPEC CamParams {
private:
  float paramf1, paramf2, paramox, paramoy, paramk1, paramk2, paramk3, paramk4;
public:
  CamParams(float f1, float f2, float ox, float oy, 
            float k1, float k2, float k3, float k4) {
    paramf1 = f1;
    paramf2 = f2;
    paramox = ox;
    paramoy = oy;
    paramk1 = k1;
    paramk2 = k2;
    paramk3 = k3;
    paramk4 = k4;
  }
  float f1() { return paramf1; }
  float f2() { return paramf2; }
  float ox() { return paramox; }
  float oy() { return paramoy; }
  float k1() { return paramk1; }
  float k2() { return paramk2; } 
  float k3() { return paramk3; } 
  float k4() { return paramk4; }
};

class DECLSPEC featureTrack {
  private :
    GLuint fb[10],depth_rb;
    GLuint inputfb, testTex; 

    //orientation/feature vector resources
    GLuint occlusionQueries;
    GLuint pixelCount, gaussian16x16tex;
    GLuint orientationfb, orientationQuadTex[5], orientationSumTex; 
    GLuint orientationTex, orientationfb2;
    GLuint featureCoordsLUTTex;

    GLuint featurefb, featureTex[16];
    float *featureCoordsBuf, *featureCoordsTex;
    unsigned char *stencilbuf;
    float *orientationsBuf;
    GLuint featureWorkBufs[4], featureWorkTex[16];
	
	//to take the place of pBuffer
	GLuint descTex;

    /* vars for Cg */
    CGcontext cgContext;
    CGprogram undistortProgram, gaussianderivXProgram, magdirProgram,
              gaussianderivYProgram,
              cannysearchProgram, dxdyProgram, gaussianXProgram, gaussianYProgram,
              derandomProgram, decideProgram, basicProgram;

    /* Cg for feature vector computations */
    CGprogram passredProgram, orientationProgram, featureProgram, sum4texProgram;

    CGprofile cgProfile;
    
 
    int numCorners; // = 0;
    float featureBuf0[16*640*4];

//    float cannythresh[4] = {2.0, 4.0, 4.0, 4.0};
    float cannythresh[4] ;
    float derandomthresh[4];// = {0.5, 0.25, 0.25, 0.25};
    float cornerthresh[4];// = {11.0, 0.25, 0.25, 0.25};


    unsigned char rbbuf[640*480*4];

    int Width;
    int Height;
    float depth;

    static GLenum errCode;
    const GLubyte *errString;

    void reshape(int,int);
    void reshapeDescFBO(int,int);
    void drawQuadTex();
    void drawQuadFBT();
    void makeLUT(unsigned char *f);
    CGprogram load_cgprogram(CGprofile prof, char *name);
    void init_cg();
    void locateFeatures( GLuint texIn );
    void makeLookupTexture();
	void makeLookupTextureNoStencil();
    void calcOrientations();
    void calcFeatures();
    void createScene( int num, float *coords, float *orients, float *buf, Scene &s );
    void render_redirect(GLuint texIn);

  public : 
    GLuint tex[10];   ///< OpenGL textures which hold intermediate results.  Can be 
                      ///< used to view intermediate processing.

	
    /// The constructor for the feature track object is passed in the width and height of the
    /// images upon which it will operate. 
 
	featureTrack(int width,   ///<  width of the images upon which it will operate.
                 int height); ///<  height of the images upon which it will operate.
 
    ///  Returns a pointer to a newly created scene from the input texture image.
    ///  When the returned scene is no longer needed it should be deleted.
    void getScene( GLuint texIn, Scene &s );
    //Scene *getScene( GLuint texIn );
    ///Get the width of images upon which the feature track object is set to operate
    int width() { return Width; } 
    ///Get the height of images upon which the feature track object is set to operate
    int height() { return Height; }

    void setCamParams(CamParams cp);
};

Matches DECLSPEC findMatches( Scene &s0,
                      Scene &s1,
                      float magRadius ,  
                      float improvement  ) ;
double DECLSPEC distSq( Coords &c0, Coords &c1 ) ;
Matches DECLSPEC &findMatchesByPosition( Scene &s0,
                                Scene &s1,
                                float radius,
                                PCT P, 
                                int width, 
                                int height );


#ifndef WIN32
#include <openvidia/openvidia_lnx.h>
#endif

#endif

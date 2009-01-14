#ifndef __DFT_CC
#define __DFT_CC

#include <GL/gl.h>
#include <GL/glut.h>
#include <Cg/cgGL.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <openvidia/libopenvidia.h>

#define PI 3.14159265358979

enum dftDirectionType  { DFT_FORWARD, DFT_REVERSE };
enum DataMajorType  { DFT_ROW_VECTOR, DFT_COLUMN_VECTOR };

class DFT : public FBO_Filter {
private :
    // make Wn, sinusoids (cos,sin pairs) to use in DFT
    void make_Wn( int N, float *RGBAbuffer ) {
        float *ptr = RGBAbuffer;
        for ( int u=0; u<N ; u++ ) {
            for ( int x=0; x<N ; x++ ) {
                *ptr = cosf( 2*PI/N*u*x ) ;
                ptr++;

                *ptr = -1.0*sinf( 2*PI/N*u*x ) ;
                ptr++;

                *ptr = 0.0;
                ptr++;

                *ptr = 0.0;
                ptr++;


            }
        }
    }

protected :

    int _N, _M;
    GLuint WnTex;
    DataMajorType _datamajor;

    CGprogram dftForwardProgram, dftReverseProgram ;

public :

    DFT(int N, int M, GLuint oTex, DataMajorType datamajor )
            : FBO_Filter( CG_PROFILE_FP30, NULL , oTex, M, N )
    {
        float *Wn_buffer = (float *)malloc( sizeof(float)*4 * N  *N );
        make_Wn( N, Wn_buffer );
        glGenTextures( 1, &WnTex );
        glBindTexture(GL_TEXTURE_RECTANGLE_NV, WnTex );
        glTexImage2D( GL_TEXTURE_RECTANGLE_NV, 0, GL_FLOAT_RG32_NV, N, N, 0, GL_RGBA,
                      GL_FLOAT, Wn_buffer );
        free(Wn_buffer);
        _N = N;
        _M = M;
        _datamajor = datamajor ;

        // we need to specify the size of the DFT for the DFT fragment program. So,
        // do this when it is compiled.  destroy the existing version and instead
        // create another/
        if ( cgProgram != 0 ) cgDestroyProgram( cgProgram );

        char *dftsizedef = (char *)malloc( 16 );
        char *dftdirection = (char *)malloc( 24 ) ;
        char *dftmajor = (char *)malloc( 24 ) ;
        char *args[4];
        sprintf(dftsizedef, "-DDFTN=%d", N );
        sprintf(dftdirection, "-DDFT_FORWARD");
        ( datamajor == DFT_COLUMN_VECTOR ? sprintf(dftmajor, "-DDFT_COLUMN_VECTOR") :
          sprintf(dftmajor, "-DDFT_ROW_VECTOR") );
        args[0] = dftsizedef;
        args[1] = dftdirection;
        args[2] = dftmajor;
        args[3] = '\0' ;
        dftForwardProgram  = load_cgprogram(cgProfile, "FP-DFT.cg", args );
        cgGLLoadProgram( dftForwardProgram );

        sprintf(dftdirection, "-DDFT_REVERSE");
        args[1] = dftdirection;
        dftReverseProgram  = load_cgprogram(cgProfile, "FP-DFT.cg", args );
        cgGLLoadProgram( dftReverseProgram );


    }



    ///TODO: make the float pointer an LBUffer to ensure proper sizing.
    GLuint apply( GLuint iTex, bool FBOtex, dftDirectionType dftDir = DFT_FORWARD,
                  float *rbbuf = NULL )
    {
        glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, fb );
        glEnable(GL_TEXTURE_RECTANGLE_NV);

        glActiveTextureARB( GL_TEXTURE0_ARB );
        glBindTexture( GL_TEXTURE_RECTANGLE_NV, WnTex );

        glActiveTextureARB( GL_TEXTURE1_ARB );
        glBindTexture( GL_TEXTURE_RECTANGLE_NV, iTex );

        renderBegin();
        cgGLEnableProfile(cgProfile);
        switch (dftDir) {
        case DFT_FORWARD :
            cgGLBindProgram(dftForwardProgram);
            break;
        case DFT_REVERSE :
            cgGLBindProgram(dftReverseProgram);
            break;
        default :
            assert(0);
            break;
        }
        ( FBOtex ? drawQuadFBT() : drawQuadTex() );
        cgGLDisableProfile(cgProfile);
        renderEnd();

        if ( rbbuf != NULL ) {
            glReadPixels(0,0, _M,_N, GL_RGB, GL_FLOAT, rbbuf );
        }

        glActiveTextureARB( GL_TEXTURE0_ARB );
        glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0);

        return oTex ;
    }

};

#endif

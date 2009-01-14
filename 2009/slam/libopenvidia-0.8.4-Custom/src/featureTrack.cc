/**
 * This file is part of the OpenVIDIA project at http://openvidia.sf.net
 * Copyright (C) 2004, James Fung
 *
 * OpenVIDIA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * OpenVIDIA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenVIDIA; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **/
#ifdef WIN32
#	include <GL/glew.h>
#endif
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <assert.h>
#include <Cg/cgGL.h>

#include <openvidia/openvidia32.h>

#include "errutil.h"
#include "taps.h" //gaussian 16x16 filter taps
using namespace std;

#include "unpack.h"

#define PIPELEN 10

extern GLenum errCode;
extern const GLubyte *errString;

const GLenum buffers[] = {
    GL_COLOR_ATTACHMENT0_EXT,
    GL_COLOR_ATTACHMENT1_EXT
};

const GLenum attachmentBuffers[] = {
    GL_COLOR_ATTACHMENT0_EXT,
    GL_COLOR_ATTACHMENT1_EXT,
    GL_COLOR_ATTACHMENT2_EXT,
    GL_COLOR_ATTACHMENT3_EXT,
    GL_COLOR_ATTACHMENT4_EXT,
    GL_COLOR_ATTACHMENT5_EXT
};

void featureTrack::reshape(int w, int h)
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float scale = 1.0;

    glFrustum(0.0, 1.0/scale,  0.0/scale, 1.0/scale,   1.0/scale,   100.0);

    gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  depth,   0.0,   1.0, 0.0);

    glMatrixMode(GL_MODELVIEW);


    /*
    fpbuffer.activate();

      glClearColor (0.0, 0.0, 0.0, 0.0);
      glViewport(0, 0, (GLsizei) w, (GLsizei) 16);
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
     // glFrustum(0.0, 1.0,  1.0, 0.0,   1.0,   100.0);
      glOrtho(0.0, 1.0,  1.0, 0.0,   1.0,   100.0);
      gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  depth,   0.0, 1.0, 0.0);

      glMatrixMode(GL_MODELVIEW);
    fpbuffer.deactivate();
    */


    glLoadIdentity();
}

void featureTrack::reshapeDescFBO(int w, int h ) {
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glViewport(0, 0, (GLsizei) w, (GLsizei) 16);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
// glFrustum(0.0, 1.0,  1.0, 0.0,   1.0,   100.0);
    glOrtho(0.0, 1.0,  1.0, 0.0,   1.0,   100.0);
    gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  depth,   0.0, 1.0, 0.0);

    glMatrixMode(GL_MODELVIEW);
}



void featureTrack::drawQuadTex()
{
    glBegin(GL_QUADS);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glVertex3f(0.0, 1.0, depth);


    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glVertex3f(1.0, 0.0, depth);

    glEnd();

}

void featureTrack::drawQuadFBT()
{

    glBegin(GL_QUADS);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glVertex3f(0.0, 1.0, depth);


    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glVertex3f(1.0, 0.0, depth);

    glEnd();


}



void featureTrack::makeLUT(unsigned char *f)  {
    numCorners = 0;
    for (int i=0; i<Height ; i++ ) {
        int rowoffset = i*Width;
        for (int j=0; j<Width ; j++ ) {
            int pos = numCorners*4;
            if ( f[ (rowoffset+j) ]  != 0 ) {
                featureCoordsTex[pos] = j;
                featureCoordsTex[pos+1] = i;
                featureCoordsTex[pos+2] = 0.0;
                featureCoordsTex[pos+3] = 0.0;
                numCorners++;
                if ( numCorners >= Width ) goto endit;
            }
        }
    }
endit:
    //fprintf(stderr, "packed %d\n", numCorners);

    glBindTexture( GL_TEXTURE_RECTANGLE_NV, featureCoordsLUTTex);
    glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV,  0, 0,0, Width, 1,
                    GL_RGBA, GL_FLOAT,  featureCoordsTex );

    ERRCHECK()
}

void featureTrack::render_redirect( GLuint texIn ) {

    reshape(Width,Height);
    cgGLEnableProfile(cgProfile);

    //places feature output in tex[9]
    locateFeatures(texIn);
    // @ 43 msecs, GoForce6600, 640 features.

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

    //fills in featureCoordLUTTex
    //
    // featureCoordsLUTTex is a 1d array with (x,y) feature coordinates
    // in its red,green values.
    //
    makeLookupTexture();


    // @ 50 msecs, GoForce 6600, 640 features.

    //fills in orientationTex
    //
    //  orientationTex corresponds 1to1 with each of the features in
    //  featureCoordsLUTTex.  orientations are stored as {dx,dy,atan2(dx,dy),-}
    //

    calcOrientations();
    // @ 52.6 msecs, GoForce 6600 640 features,

    //fills in featureWorkTex[16]
    //
    // these 16 textures are widthx1, and each holds packed 8 16-bit histograms
    //  together all 16 make the 128 element feature vector
    calcFeatures();
    // @ 58.8 msecs, GoForce 6600 640 features
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

}

CGprogram featureTrack::load_cgprogram(CGprofile prof, char *name) {
    fprintf(stderr, "loading %s\n", name);
    return cgCreateProgramFromFile( cgContext, CG_SOURCE,
                                    name, prof, "FragmentProgram", 0);
}

/*
 * A callback function for cg to use when it encounters an error
 */
static CGcontext errContext;

static void cgErrorCallback(void) {
    CGerror LastError = cgGetError();

    if (LastError)
    {
        const char *Listing = cgGetLastListing(errContext);
        printf("\n---------------------------------------------------\n");
        printf("%s\n\n", cgGetErrorString(LastError));
        printf("%s\n", Listing);
        printf("---------------------------------------------------\n");
        printf("Cg error, exiting...\n");
        exit(0);
    }
}


void featureTrack::init_cg() {


    cgProfile = CG_PROFILE_FP40;
    cgContext = cgCreateContext();
    errContext = cgContext;
    cgSetErrorCallback( cgErrorCallback) ;

    undistortProgram = load_cgprogram(CG_PROFILE_FP40, "FP-func-undistort.cg");
    gaussianderivXProgram= load_cgprogram(CG_PROFILE_FP40, "FP-gaussianderiv-sigma1-x.cg");
    gaussianderivYProgram= load_cgprogram(CG_PROFILE_FP40, "FP-gaussianderiv-sigma1-y.cg");
    magdirProgram = load_cgprogram(CG_PROFILE_FP40, "FP-magdir.cg");
    cannysearchProgram = load_cgprogram(CG_PROFILE_FP40, "FP-canny-search-with-corners.cg");
    gaussianXProgram = load_cgprogram(CG_PROFILE_FP40, "FP-gaussian-sigma3-x-rgb.cg");
    gaussianYProgram = load_cgprogram(CG_PROFILE_FP40, "FP-gaussian-sigma3-y-rgb.cg");
    derandomProgram  = load_cgprogram(CG_PROFILE_FP40, "FP-derandom-corners.cg");
    decideProgram    = load_cgprogram(CG_PROFILE_FP40, "FP-decide.cg");
    basicProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-basic.cg");


    cgGLLoadProgram( undistortProgram );
    cgGLLoadProgram( gaussianderivXProgram );
    cgGLLoadProgram( gaussianderivYProgram );
    cgGLLoadProgram( magdirProgram );
    cgGLLoadProgram( cannysearchProgram );
    cgGLLoadProgram( gaussianXProgram );
    cgGLLoadProgram( gaussianYProgram );
    cgGLLoadProgram( derandomProgram );
    cgGLLoadProgram( decideProgram );
    cgGLLoadProgram( basicProgram );

    cgGLSetParameter4fv( cgGetNamedParameter(cannysearchProgram, "thresh"),
                         cannythresh );
    cgGLSetParameter4fv( cgGetNamedParameter(derandomProgram, "thresh"),
                         derandomthresh );
    cgGLSetParameter4fv( cgGetNamedParameter(decideProgram, "thresh"),
                         cornerthresh );

    /* programs which deal with the feature vector computation */
    passredProgram     = load_cgprogram(CG_PROFILE_FP30, "FP-pass-red.cg");
    featureProgram     = load_cgprogram(CG_PROFILE_FP40, "FP-feature.cg");
    orientationProgram     = load_cgprogram(CG_PROFILE_FP30, "FP-orientation.cg");
    cgGLLoadProgram(sum4texProgram=load_cgprogram(CG_PROFILE_FP30, "FP-sum4tex.cg"));

    cgGLLoadProgram( passredProgram );
    cgGLLoadProgram( featureProgram );
    cgGLLoadProgram( orientationProgram );

}



featureTrack::featureTrack(int W, int H)  {
    Width = W;
    Height = H;
    depth = -1.0;
    /*  XXX
       if( glutGet(GLUT_WINDOW_STENCIL_SIZE) == 0 ) {
         cerr<<"[featureTrack] **ERROR** A stencil is needed.  Try glutInitDisplay(..|GLUT_STENCIL...)"<<endl;
         exit(1);
       }
    */
    cannythresh[0] = 2.0;
    cannythresh[1] = 4.0;
    cannythresh[2] = 4.0;
    cannythresh[3] = 4.0;

    derandomthresh[0] = 0.5;
    derandomthresh[1] = 0.25;
    derandomthresh[2] = 0.25;
    derandomthresh[3] = 0.25;

    cornerthresh[0] = 11.0;
    cornerthresh[1] = 0.25;
    cornerthresh[2] = 0.25;
    cornerthresh[3] = 0.25;

    numCorners = 0;

    int i=0;
    int mca = -1;
    glGetIntegerv( GL_MAX_COLOR_ATTACHMENTS_EXT, &mca );
    fprintf(stderr, "mca = %d\n", mca );

//fpbuffer.create(Width,16);
//procbuffer.create(width, height);

//procbuffer.activate();
    reshape(Width, Height );
    glGenTextures(1, &testTex );
    glGenTextures(1, &gaussian16x16tex );
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, gaussian16x16tex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_FLOAT_RGBA16_NV, 16,16, 0,
                 GL_RED, GL_FLOAT, taps);
    ERRCHECK()

    //for counting how many features we have
    glGenOcclusionQueriesNV(1, &occlusionQueries);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glEnable(GL_TEXTURE_RECTANGLE_NV);
    glClearDepth(  1.0);
    glClearColor(0.0, 0.0, 1.0, 1.0);

    //gen the frambuffer object (FBO), similar manner as  a texture
    glGenFramebuffersEXT(PIPELEN, fb);
    glGenFramebuffersEXT(1, &inputfb);
    ERRCHECK()
    // make a texture
    glGenTextures(PIPELEN, tex);              // texture
    ERRCHECK()

    //undistortion can be a 8bit precision i suppose

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[0]);
    glEnable(GL_TEXTURE_RECTANGLE_NV);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[0]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_RGBA,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[0], 0);

    CHECK_FRAMEBUFFER_STATUS()


    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[4]);
    glEnable(GL_TEXTURE_RECTANGLE_NV);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[4]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[4], 0);

    //canny-search..+dxdy
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[5]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_NV, tex[5], 0);
    CHECK_FRAMEBUFFER_STATUS()
    ERRCHECK()
    CHECK_FRAMEBUFFER_STATUS()

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[1]);
    glEnable(GL_TEXTURE_RECTANGLE_NV);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[1]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[1], 0);
    CHECK_FRAMEBUFFER_STATUS();
    ERRCHECK()

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[2]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);

    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_NV, tex[2], 0);
    CHECK_FRAMEBUFFER_STATUS();

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[3]);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);

    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT2_EXT, GL_TEXTURE_RECTANGLE_NV, tex[3], 0);
    CHECK_FRAMEBUFFER_STATUS();


    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[6]);

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[6]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[6], 0);

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[7]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_NV, tex[7], 0);

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[8]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT2_EXT, GL_TEXTURE_RECTANGLE_NV, tex[8], 0);

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[9]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,Height,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT3_EXT, GL_TEXTURE_RECTANGLE_NV, tex[9], 0);


    CHECK_FRAMEBUFFER_STATUS();



    glGenRenderbuffersEXT(1, &depth_rb); // render buffer
// initialize depth renderbuffer
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[4]);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depth_rb);
    glRenderbufferStorageEXT (GL_RENDERBUFFER_EXT,
                              GL_DEPTH_COMPONENT24, Width, Height);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depth_rb);
    CHECK_FRAMEBUFFER_STATUS();



    //
    //orientation
    //
    // use a single framebuffer with multiple color attachements.
    // each rendeirn pass will render to a different color output buffer.

    glGenTextures(4, orientationQuadTex);              // texture
    glGenTextures(1, &featureCoordsLUTTex);              // texture
    glGenTextures(1, &orientationTex);              // texture
    glGenTextures(1, &orientationSumTex);              // texture

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, featureCoordsLUTTex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,1,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationTex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA32_NV,Width,1,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
//procbuffer.deactivate();

///pbuffer->FBO
//fpbuffer.activate();
    reshapeDescFBO(Width,16);
    glGenFramebuffersEXT(1, &orientationfb2);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, orientationfb2);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationSumTex);
    glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,16,0,
                 GL_RGBA, GL_FLOAT,NULL);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, attachmentBuffers[0],
                              GL_TEXTURE_RECTANGLE_NV, orientationSumTex, 0);
    CHECK_FRAMEBUFFER_STATUS()


    glGenFramebuffersEXT(1, &orientationfb);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, orientationfb);
    for ( i=0; i<4; i++ ) {
        glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[i]);
        glEnable(GL_TEXTURE_RECTANGLE_NV);
        glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA16_NV,Width,16,0,
                     GL_RGBA, GL_FLOAT,NULL);
        glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, attachmentBuffers[i],
                                  GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[i], 0);
        CHECK_FRAMEBUFFER_STATUS()
    }
    ERRCHECK()
    glGenFramebuffersEXT(4, featureWorkBufs);
    glGenTextures( 16, featureWorkTex );
    int counter = 0;
    for ( int j=0; j<4 ; j++ ) {
        glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, featureWorkBufs[j] );
        for ( i=0 ; i<4 ; i++ ) {
            glBindTexture(GL_TEXTURE_RECTANGLE_NV, featureWorkTex[counter]);
            glEnable(GL_TEXTURE_RECTANGLE_NV);
            glTexImage2D(GL_TEXTURE_RECTANGLE_NV,0,GL_FLOAT_RGBA32_NV,Width,16,0,
                         GL_RGBA, GL_FLOAT,NULL);
            glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
            glTexParameteri(GL_TEXTURE_RECTANGLE_NV,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
            glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, attachmentBuffers[i],
                                      GL_TEXTURE_RECTANGLE_NV, featureWorkTex[counter], 0);
            CHECK_FRAMEBUFFER_STATUS()
            counter++;
        }
    }
    ERRCHECK()
//pbuffer->FBO
//fpbuffer.deactivate();
    reshape(Width,Height);
//procbuffer.activate();
    //'unbind' the frambuffer object, so subsequent drawing ops are not
    // drawn into the FBO.
    // '0' means "windowing system provided framebuffer
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);


    init_cg();

//procbuffer.deactivate();
    featureCoordsBuf = (float *)malloc( Width*Height*sizeof(float)*4 );
    orientationsBuf = (float *)malloc( Width*sizeof(float)*4 );
    featureCoordsTex = (float *)malloc( Width*sizeof(float)*4 );
    stencilbuf = (unsigned char*)malloc( Width*Height );
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

}

void featureTrack::locateFeatures( GLuint texIn ) {

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[0]);
    glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[0], 0);
    glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT);

    cgGLBindProgram(undistortProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, texIn);
    CHECK_FRAMEBUFFER_STATUS()
    drawQuadTex();

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[1]);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[1], 0);
    glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT);
    CHECK_FRAMEBUFFER_STATUS()
    cgGLBindProgram(gaussianderivXProgram);

    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[0]);

    glBegin(GL_QUADS);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+2.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+3.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-2.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f-3.0, 0);

    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width+1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width+2.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width+3.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width-1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width-2.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width-3.0 , 0);

    glVertex3f(1.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width+1.0 , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width+2.0 , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width+3.0 , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width-1.0 , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width-2.0 , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width-3.0 , (float)Height);

    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+2.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+3.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-2.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f-3.0, (float)Height);

    glVertex3f(0.0, 1.0, depth);

    glEnd();


    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[2]);
    glDrawBuffer( GL_COLOR_ATTACHMENT1_EXT);
    //   glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
    //            GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[2], 0);
    cgGLBindProgram(gaussianderivYProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[0]);
    drawQuadFBT();

    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[3]);
    glDrawBuffer( GL_COLOR_ATTACHMENT2_EXT);
    //   glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
    //            GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[3], 0);
    cgGLBindProgram(magdirProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[1]);
    glActiveTextureARB(GL_TEXTURE1_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[2]);

    glBegin(GL_QUADS);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, 0);
    glVertex3f(0.0, 0.0, depth);


    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width , 0);

    glVertex3f(1.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width , (float)Height);

    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, (float)Height);
    glVertex3f(0.0, 1.0, depth);


    glEnd();


    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[4]);

    //glDisable(GL_DEPTH_TEST);
    //glClearDepth(  1.0);
    //glDepthFunc(GL_LESS );
    //glClear( GL_DEPTH_BUFFER_BIT);
    //glDepthMask(GL_TRUE);

    ///??? WHY is the framebuffer object "forgetting" its bindings from the
    /// initialization??
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[4], 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_NV, tex[5], 0);
    glDrawBuffers(2, buffers);
    CHECK_FRAMEBUFFER_STATUS()

    cgGLBindProgram(cannysearchProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[3]);
    glBegin(GL_QUADS);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+1.0, 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+0.0, 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-1.0, 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f-0.0, 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+1.0, 0-1.0);

    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width+1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width+1.0 , 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width+0.0 , 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width-1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width-1.0 , 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width     , 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width+1.0 , 0-1.0);

    glVertex3f(1.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width +1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width +1.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width +0.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width -1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width -1.0, (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width     , (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width +1.0, (float)Height-1.0);
    glVertex3f(1.0, 1.0, depth);


    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+1.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+0.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-1.0, (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f    , (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+1.0, (float)Height-1.0);
    glVertex3f(0.0, 1.0, depth);


    glEnd();

//glDepthMask(GL_FALSE);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[6]);

    //glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LEQUAL);
    //glDepthMask(GL_FALSE);

    cgGLBindProgram(gaussianXProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[5]);
    glDrawBuffer( GL_COLOR_ATTACHMENT0_EXT );
//  drawQuad();
    // 28 msecs to here ( 12 msecs for the stage )
    glBegin(GL_QUADS);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+2.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+3.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f+4.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f+5.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f+6.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+7.0, 0);

    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width+1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width+2.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width+3.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width+4.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width+5.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width+6.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width+7.0 , 0);

    glVertex3f(1.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width +1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width +2.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width +3.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width +4.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width +5.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width +6.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width +7.0, (float)Height);
    glVertex3f(1.0, 1.0, depth);


    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+2.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+3.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f+4.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f+5.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f+6.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+7.0, (float)Height);
    glVertex3f(0.0, 1.0, depth);


    glEnd();

    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[7]);

    // glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
    //          GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[7], 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT1_EXT);
    //glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LEQUAL);
    //glDepthMask(GL_FALSE);

    cgGLBindProgram(gaussianYProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[6]);
    glBegin(GL_QUADS);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0, 0.0f);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0, 0.0f+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0, 0.0f+2.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0, 0.0f+3.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0, 0.0f+4.0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0, 0.0f+5.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0, 0.0f+6.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0, 0.0f+7.0);
    glVertex3f(0.0, 0.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width, 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width, 0+2.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width, 0+3.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width, 0+4.0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width, 0+5.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width, 0+6.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width, 0+7.0);
    glVertex3f(1.0, 0.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width , (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width , (float)Height+2.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width , (float)Height+3.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width , (float)Height+4.0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width , (float)Height+5.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width , (float)Height+6.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width , (float)Height+7.0);
    glVertex3f(1.0, 1.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f, (float)Height+2.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f, (float)Height+3.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f, (float)Height+4.0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f, (float)Height+5.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f, (float)Height+6.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f, (float)Height+7.0);
    glVertex3f(0.0, 1.0, depth);
    glEnd();


    //glDisable(GL_DEPTH_TEST);

    //glDepthMask(GL_TRUE);
    // 40 msecs to here (12 msecs for the stage )

    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[8]);
    //   glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
    //            GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[8], 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT2_EXT);
    cgGLBindProgram(derandomProgram);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[7]);
    glActiveTextureARB(GL_TEXTURE1_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[4]);
    glActiveTextureARB(GL_TEXTURE2_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[3]);
    drawQuadFBT();

    //glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fb[9]);
    //   glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
    //            GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, tex[9], 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT3_EXT);
    cgGLBindProgram(decideProgram);
//driver 1.0-8174 is "forgetful" so have to re-set the params.... jf 7 dec 2005
    cgGLSetParameter4fv( cgGetNamedParameter(decideProgram, "thresh"),
                         cornerthresh );
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[8]);
    //drawQuad();
    glBegin(GL_QUADS);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+1.0, 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+0.0, 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-1.0, 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f-0.0, 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+1.0, 0-1.0);
    glVertex3f(0.0, 0.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , 0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width+1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width+1.0 , 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width+0.0 , 0+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width-1.0 , 0);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width-1.0 , 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width     , 0-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width+1.0 , 0-1.0);
    glVertex3f(1.0, 0.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width , (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width +1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width +1.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width +0.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, (float)Width -1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, (float)Width -1.0, (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, (float)Width     , (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, (float)Width +1.0, (float)Height-1.0);
    glVertex3f(1.0, 1.0, depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f+1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f+1.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f+0.0, (float)Height+1.0);
    glMultiTexCoord2fARB(GL_TEXTURE4_ARB, 0.0f-1.0, (float)Height);
    glMultiTexCoord2fARB(GL_TEXTURE5_ARB, 0.0f-1.0, (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE6_ARB, 0.0f    , (float)Height-1.0);
    glMultiTexCoord2fARB(GL_TEXTURE7_ARB, 0.0f+1.0, (float)Height-1.0);
    glVertex3f(0.0, 1.0, depth);
    glEnd();

}

void featureTrack::makeLookupTexture() {
    //clear stencil
    glClear(GL_STENCIL_BUFFER_BIT);
    glEnable(GL_STENCIL_TEST);

    // All drawing commands fail the stencil test, and are not
    // drawn, but increment the value in the stencil buffer.
    glStencilFunc(GL_NEVER, 0x0, 0x0);
    glStencilOp(GL_INCR, GL_INCR, GL_INCR);

    //only fragments which are red (feature flag) will pass, since
    //passredProgram uses discard on non red fragments
    cgGLBindProgram(passredProgram);


    glEnable(GL_TEXTURE_RECTANGLE_NV);
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[9]);

    drawQuadFBT();

    cgGLDisableProfile(cgProfile);

    // Now, allow drawing, except where the stencil pattern is 0x1
    // and do not make any further changes to the stencil buffer
    glStencilFunc(GL_EQUAL, 0x1, 0x1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glColor4f(0.0,0.0,1.0,1.0);
    glBeginOcclusionQueryNV(occlusionQueries);
    drawQuadFBT();
    glEndOcclusionQueryNV();
    //glGetOcclusionQueryuivNV(occlusionQueries, GL_PIXEL_COUNT_NV, &pixelCount);
    //fprintf(stderr, "pixel count %d\n", pixelCount );
    glReadPixels(0,0,Width,Height,GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, stencilbuf);
    glDisable(GL_STENCIL_TEST);
    makeLUT(stencilbuf);
}

//
//
//   calculate orientations
//
//   upon entry it needs the featureCoordsLUTTex to be calc'd
//
void featureTrack::calcOrientations()
{
//fpbuffer.activate();
//pbuffer->FBO
    reshapeDescFBO(Width, 16);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, orientationfb);
    reshapeDescFBO(Width, 16);
    cgGLDisableProfile(cgProfile);
    cgGLEnableProfile(CG_PROFILE_FP30);
    cgGLBindProgram(orientationProgram);

    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, featureCoordsLUTTex);
    glActiveTextureARB(GL_TEXTURE1_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[1]);
    glActiveTextureARB(GL_TEXTURE2_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[2]);
    glActiveTextureARB(GL_TEXTURE3_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, gaussian16x16tex );

    //do orientations.  texcoordO is upper left hand corner of 8x8
    // quadrant in question (of 16x16 region), texcoord1 is the
    // pixel centre itself, texcoord2 is location in a 16x16 gaussian taps tex
    // gaussian taps need to be fixored.

    //could also use a stencil to prevent empty computations!
    //hardcoding  taps values into 4 separate progs (with unrolling) would save
    // looking up into the tapsTexture.
    int outputBuffer = 0;
    for ( int xoffset=-1 ; xoffset<1 ; xoffset++ ) {
        for ( int yoffset=-1 ; yoffset<1 ; yoffset++ ) {

            glDrawBuffer( attachmentBuffers[outputBuffer++] );

            ERRCHECK()

            float offsets[4] = {(float)xoffset*8.0, (float)yoffset*8.0,
                                (float)(xoffset+1)*8.0, (float)(yoffset+1)*8.0
                               };

            cgGLSetParameter4fv( cgGetNamedParameter(orientationProgram, "offsets"),
                                 offsets );

            //need to set offset parameter based on ...ofset
            glBegin(GL_LINES);

            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
            glVertex3f(0.0, 0.5/16.0, depth);

            //glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, 0);
            //glVertex3f(1.0, 0.5/16.0, depth);
            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)numCorners, 0);
            glVertex3f((float)numCorners/(float)Width, 0.5/16.0, depth);


            glEnd();
            /*
              {
              glReadBuffer( attachmentBuffers[outputBuffer-1] );
              float result[4] = {0.0, 0.0, 0.0, 0.0};
              glReadPixels(  30, 16-1, 1,1, GL_RGBA, GL_FLOAT, result );
              fprintf(stderr, "result [ %f %f %f %f ] \n", result[0], result[1], result[2], result[3] );
              }
            */


            ERRCHECK()

        }
    }
    ERRCHECK()
    //glFinish();
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, orientationfb2);
    glDrawBuffer( attachmentBuffers[0] );
    cgGLBindProgram(sum4texProgram);
    ERRCHECK()

    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[0]);

    glActiveTextureARB(GL_TEXTURE1_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[1]);

    glActiveTextureARB(GL_TEXTURE2_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[2]);

    glActiveTextureARB(GL_TEXTURE3_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationQuadTex[3]);

    glBegin(GL_LINES);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, 0.0f, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, 0.0f, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, 0.0f, (float)16.0);
    glVertex3f(0.0, 0.5/16.0, depth);
    /*
        glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, (float)16.0);
        glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)Width, (float)16.0);
        glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)Width, (float)16.0);
        glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)Width, (float)16.0);
        glVertex3f(1.0, 0.5/16.0, depth);
    */
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)numCorners, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)numCorners, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)numCorners, (float)16.0);
    glMultiTexCoord2fARB(GL_TEXTURE3_ARB, (float)numCorners, (float)16.0);
    glVertex3f((float)numCorners/(float)Width, 0.5/16.0, depth);

    glEnd();

    glBindTexture( GL_TEXTURE_RECTANGLE_NV, orientationTex );
    glCopyTexSubImage2D( GL_TEXTURE_RECTANGLE_NV, 0, 0,0, 0, 16-1, Width, 1);
    /*
      {
      float result[4] = {0.0, 0.0, 0.0, 0.0};
      glReadPixels(  30, 16-1, 1,1, GL_RGBA, GL_FLOAT, result );
      fprintf(stderr, "--> result [ %f %f %f %f ] \n",
              result[0], result[1], result[2], result[3] );
      }
    */



//fpbuffer.deactivate();
//pbuffer->FBO
    reshape(Width,Height);

}


void featureTrack::calcFeatures()
{

//pbuffer->FBO
//fpbuffer.activate();
    reshapeDescFBO(Width,16);

    glClearColor(0.0,0.0,1.0,1.0);
    glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
    glColor4f(1.0, 1.0, 0.0, 1.0);

    cgGLEnableProfile(CG_PROFILE_FP40);
    cgGLBindProgram(featureProgram);

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, featureWorkBufs[0]);

    ERRCHECK()
    glActiveTextureARB(GL_TEXTURE0_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, featureCoordsLUTTex);

    //combining orientations and feature Coords into a single 3 element
    // texture would save another texture lookup !
    glActiveTextureARB(GL_TEXTURE1_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, orientationTex);

    glActiveTextureARB(GL_TEXTURE2_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex[9] );

    //hardcoding taps instead would save a tex lookup
    glActiveTextureARB(GL_TEXTURE3_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, gaussian16x16tex);

    glDrawBuffer( attachmentBuffers[0] );
    //glBeginOcclusionQueryNV(occlusionQueries);
    float inc = 0.0 ;
    for ( int xoffset=-2 ; xoffset<0 ; xoffset++ ) {
        for ( int yoffset=-2 ; yoffset<2 ; yoffset++ ) {

            ERRCHECK()

            glBegin(GL_LINES);
            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 0.0 );
            glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)xoffset*4.0,
                                 (float)yoffset*4.0 );
            glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)(xoffset+2)*8.0,
                                 (float)(yoffset+2)*8.0  );
            //glVertex3f( 0.0, ((float)(4*(xoffset+2)+yoffset)+2.5)/8.0, -1.0);
            glVertex3f( 0.0, inc/16.0 , depth);

            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, 0 );
            glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)xoffset*4.0,
                                 (float)yoffset*4.0 );
            glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)(xoffset+2)*8.0,
                                 (float)(yoffset+2)*8.0  );
            //glVertex3f( 1.0, ((float)(4*(xoffset+2)+yoffset)+2.5)/8.0, -1.0);
            glVertex3f( 1.0, inc/16.0 , depth);
            glEnd();
            inc += 1.0;

            ERRCHECK()

        }

    }

    //glReadPixels(0,0,numCorners,8, GL_RGBA, GL_FLOAT, featureBuf0 );
    //glDrawBuffer( attachmentBuffers[1] );

    for ( int xoffset=0 ; xoffset<2 ; xoffset++ ) {
        for ( int yoffset=-2 ; yoffset<2 ; yoffset++ ) {

            ERRCHECK()

            glBegin(GL_LINES);
            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 0.0 );
            glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)xoffset*4.0,
                                 (float)yoffset*4.0 );
            glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)(xoffset+2)*8.0,
                                 (float)(yoffset+2)*8.0  );
            //glVertex3f( 0.0, (float)((4*(xoffset)+yoffset)+2.5)/8.0, -1.0);
            glVertex3f( 0.0, inc/16.0 ,depth);

            glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, 0 );
            //glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)numCorners, 0 );
            glMultiTexCoord2fARB(GL_TEXTURE1_ARB, (float)xoffset*4.0,
                                 (float)yoffset*4.0 );
            glMultiTexCoord2fARB(GL_TEXTURE2_ARB, (float)(xoffset+2)*8.0,
                                 (float)(yoffset+2)*8.0  );
            //glVertex3f( 1.0, (float)((4*(xoffset)+yoffset)+2.5)/8.0, -1.0);
            glVertex3f( 1.0, inc/16.0 , depth);
            // glVertex3f( (float)numCorners/(float)Width, inc/16.0 , depth);
            glEnd();
            inc += 1.0;

            ERRCHECK()
            /*
              {
              float result[4] = {0.0, 0.0, 0.0, 0.0};
              glReadPixels(  30, 0, 1,1, GL_RGBA, GL_FLOAT, result );
               //gotta unpack otherwise they come out 0
              unpack_2half(result[0], &result[1], &result[2] );
              fprintf(stderr, "result [ %lf %lf %lf %lf ] \n", result[0], result[1], result[2], result[3] );
              }
            */

        }
    }


    /* DEBUG
      {
      glReadBuffer( attachmentBuffers[0] );
      float result[4] = {0.0, 0.0, 0.0, 0.0};
      float histo[8] = {0.0};

      glReadPixels(  0, 0, 1,1, GL_RGBA, GL_FLOAT, result );
      //gotta unpack otherwise they come out 0
      unpack_2half(result[0], &histo[0], &histo[1] );
      unpack_2half(result[1], &histo[2], &histo[3] );
      unpack_2half(result[2], &histo[4], &histo[5] );
      unpack_2half(result[3], &histo[6], &histo[7] );
      cerr<<" "<<histo[0]<<" "<<histo[1]<<" "<<histo[2]<<" "<<histo[3]<<" "<<histo[4];
      cerr<<" "<<histo[5]<<" "<<histo[6]<<" "<<histo[7] <<endl;

      glReadPixels(  0, 1, 1,1, GL_RGBA, GL_FLOAT, result );
      //gotta unpack otherwise they come out 0
      unpack_2half(result[0], &histo[0], &histo[1] );
      unpack_2half(result[1], &histo[2], &histo[3] );
      unpack_2half(result[2], &histo[4], &histo[5] );
      unpack_2half(result[3], &histo[6], &histo[7] );
      cerr<<" "<<histo[0]<<" "<<histo[1]<<" "<<histo[2]<<" "<<histo[3]<<" "<<histo[4];
      cerr<<" "<<histo[5]<<" "<<histo[6]<<" "<<histo[7] <<endl;


       glReadPixels(  0, 2, 1,1, GL_RGBA, GL_FLOAT, result );
      //gotta unpack otherwise they come out 0
      unpack_2half(result[0], &histo[0], &histo[1] );
      unpack_2half(result[1], &histo[2], &histo[3] );
      unpack_2half(result[2], &histo[4], &histo[5] );
      unpack_2half(result[3], &histo[6], &histo[7] );
      cerr<<" "<<histo[0]<<" "<<histo[1]<<" "<<histo[2]<<" "<<histo[3]<<" "<<histo[4];
      cerr<<" "<<histo[5]<<" "<<histo[6]<<" "<<histo[7] <<endl;

       glReadPixels(  0, 3, 1,1, GL_RGBA, GL_FLOAT, result );
      //gotta unpack otherwise they come out 0
      unpack_2half(result[0], &histo[0], &histo[1] );
      unpack_2half(result[1], &histo[2], &histo[3] );
      unpack_2half(result[2], &histo[4], &histo[5] );
      unpack_2half(result[3], &histo[6], &histo[7] );
      cerr<<" "<<histo[0]<<" "<<histo[1]<<" "<<histo[2]<<" "<<histo[3]<<" "<<histo[4];
      cerr<<" "<<histo[5]<<" "<<histo[6]<<" "<<histo[7] <<endl;

      }
    */



    glReadPixels(0,0,Width,16, GL_RGBA, GL_FLOAT, featureBuf0 );
    /*
      unpack_2half(featureBuf0[0], &histo[0], &histo[1] );
      unpack_2half(featureBuf0[1], &histo[2], &histo[3] );
      unpack_2half(featureBuf0[2], &histo[4], &histo[5] );
      unpack_2half(featureBuf0[3], &histo[6], &histo[7] );
      fprintf(stderr, "featureHisto 0 : [ %lf %lf %lf %lf %lf %lf %lf %lf ] \n",
        histo[0], histo[1], histo[2], histo[3], histo[4],
        histo[5], histo[6], histo[7] );
    */

    //glEndOcclusionQueryNV();

    //glGetOcclusionQueryuivNV(occlusionQueries, GL_PIXEL_COUNT_NV, &pixelCount);
    //fprintf(stderr, "pixel count %d\n", pixelCount );

    glActiveTextureARB( GL_TEXTURE0_ARB );
    cgGLBindProgram(basicProgram);
    glBindTexture( GL_TEXTURE_RECTANGLE_NV, orientationTex );
    glBegin(GL_LINES);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 0.0 );
    glVertex3f( 0.0, 0.5/16.0f , depth);
    //glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)Width, 0 );
    ///glVertex3f( 1.0, 0.5/16.0f , depth);
    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)numCorners, 0 );
    glVertex3f((float)numCorners/(float)Width, 0.5/16.0f , depth);
    glEnd();
    //glReadPixels(0,16-1,Width,1, GL_RGBA, GL_FLOAT, orientationsBuf );
    glReadPixels(0,16-1,numCorners,1, GL_RGBA, GL_FLOAT, orientationsBuf );
    /*
      {
      float result[4] = {0.0, 0.0, 0.0, 0.0};
      glReadPixels(  30, 16-1, 1,1, GL_RGBA, GL_FLOAT, result );
      fprintf(stderr, "--> result [ %f %f %f %f ] \n", result[0], result[1], result[2], result[3] );
      }
    */


    cgGLDisableProfile(CG_PROFILE_FP30);
//fpbuffer.deactivate();
//pbuffer->FBO
    reshape(Width,Height);

}

void featureTrack::getScene( GLuint texIn, Scene &s )    ///< OpenGL texture object name upon which it will operate.
{
    render_redirect( texIn );
    createScene( numCorners, featureCoordsBuf, orientationsBuf, featureBuf0, s );
    return;
}



//this function is absurdly inefficient.
void featureTrack::createScene( int num, float *coords, float *orients, float *buf, Scene &s )
{
    //Scene *scene = new Scene();
    s.features.resize(num);

    int i=0 ;
    for ( i = 0 ; i<num ; i++ ) {
        float magnitude = 0.0;
        int xoffset = i*4 ;
        Feature &f = (s.features[i]);
#ifndef WIN32
        f.descriptor.clear();
        f.descriptor.reserve(128);
#endif
        for ( int j = 0 ; j<16 ; j++ ) {
            int pos = j*Width*4 + xoffset ;

            for ( int k= 0 ; k<4 ; k++ ) {
                float a,b;
                unpack_2half( buf[pos+k] ,  &a, &b );
                f.descArray[j*8+k*2] = a;
                f.descArray[j*8+k*2+1] = b;
                //must use a push_back in g++4.0 othersie the
                //descArray assignments above are not performed after -O3 optimzation.
                //also, a cerr works here, but then there has to be output
#ifndef WIN32
                f.descriptor.push_back(a);
                f.descriptor.push_back(b);
#endif
                //if( i==0 && k==0 && j==0) cerr<<".";
//        asm("nop");
//magic line.  gcc -03 wont put any data in, without the cerr above.
// probably because of unpack_2half function bizarreness.
// for now, use O2
                magnitude += a*a+b*b ;
            }
        }
        f.magnitude = (float)magnitude;
        f.dx = orientationsBuf[xoffset+0];
        f.dy = orientationsBuf[xoffset+1];
        f.set( featureCoordsTex[xoffset], featureCoordsTex[xoffset+1] );
        f.orientation = orientationsBuf[i*4+2];
    }
    return ;
}

void featureTrack::setCamParams(CamParams cp) {
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "f1"), cp.f1() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "f2"), cp.f2() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "ox"), cp.ox() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "oy"), cp.oy() );

    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "k1"), cp.k1() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "k2"), cp.k2() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "k3"), cp.k3() );
    cgGLSetParameter1f( cgGetNamedParameter(undistortProgram, "k4"), cp.k4() );
}

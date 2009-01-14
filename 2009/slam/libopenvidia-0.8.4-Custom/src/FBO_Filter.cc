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
#include <GL/glew.h>
#define GLEW_STATIC 1
#endif

#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <assert.h>
#include <Cg/cgGL.h>
#include <openvidia/openvidia32.h>

#define ERRCHECK() \
 
#define CHECK_FRAMEBUFFER_STATUS() \
{\
 GLenum status; \
 status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT); \
 switch(status) { \
 case GL_FRAMEBUFFER_COMPLETE_EXT: \
   break; \
 case GL_FRAMEBUFFER_UNSUPPORTED_EXT: \
   fprintf(stderr,"framebuffer GL_FRAMEBUFFER_UNSUPPORTED_EXT\n");\
    /* you gotta choose different formats */ \
   assert(0); \
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT: \
   fprintf(stderr,"framebuffer INCOMPLETE_ATTACHMENT\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT: \
   fprintf(stderr,"framebuffer FRAMEBUFFER_MISSING_ATTACHMENT %d\n",__LINE__);\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT: \
   fprintf(stderr,"framebuffer FRAMEBUFFER_DIMENSIONS\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT: \
   fprintf(stderr,"framebuffer INCOMPLETE_FORMATS\n");\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT: \
   fprintf(stderr,"framebuffer INCOMPLETE_DRAW_BUFFER %s:%d\n",__FILE__,__LINE__);\
   break; \
 case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT: \
   fprintf(stderr,"framebuffer INCOMPLETE_READ_BUFFER %s:%d\n",__FILE__,__LINE__);\
   break; \
 case GL_FRAMEBUFFER_BINDING_EXT: \
   fprintf(stderr,"framebuffer BINDING_EXT\n");\
   break; \
/*\
 case GL_FRAMEBUFFER_STATUS_ERROR_EXT: \
   fprintf(stderr,"framebuffer STATUS_ERROR\n");\
   break; \
 this one got left out of /usr/include/GL/glext.h v7667 nvidia drivers?\
*/\
 default: \
   /* programming error; will fail on all hardware */ \
   assert(0); \
 }\
}

//#include <openvidia/errutil.h>

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


CGprogram FBO_Filter::load_cgprogram(CGprofile prof, char *name, char **args)
{
    fprintf(stderr, "loading %s\n", name);
    return cgCreateProgramFromFile( cgContext, CG_SOURCE,
                                    name, prof, "FragmentProgram", (const char **)args);
}

void FBO_Filter::renderBegin()
{
    //since we might be operating on downsampled images, we need
    // to reshape our current drawing area. Record the previous
    // settings first though.

    glGetIntegerv(GL_VIEWPORT, previousViewportDims );

    glViewport(0, 0, (GLsizei) tWidth, (GLsizei) tHeight );
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    float scale = 1.0;
    glFrustum(0.0, 1.0/scale,  0.0/scale, 1.0/scale,   1.0/scale,   100.0);
    gluLookAt(0.0,0.0,0.0,  0.0, 0.0,  depth,   0.0,   1.0, 0.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}

//return the rendering state to what it was before.
void FBO_Filter::renderEnd()
{
    glViewport( previousViewportDims[0], previousViewportDims[1],
                previousViewportDims[2], previousViewportDims[3] );
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void FBO_Filter::drawQuadTex()
{
    glBegin(GL_QUADS);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)tHeight);
    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glVertex3f(0.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)tWidth , 0);
    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)tWidth , (float)tHeight);
    glVertex3f(1.0, 0.0, depth);

    glEnd();
}

void FBO_Filter::drawQuadFBT()
{
    glBegin(GL_QUADS);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, 0);
    glVertex3f(0.0, 0.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0f, (float)tHeight);
    glVertex3f(0.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)tWidth , (float)tHeight);
    glVertex3f(1.0, 1.0, depth);

    glMultiTexCoord2fARB(GL_TEXTURE0_ARB, (float)tWidth , 0);
    glVertex3f(1.0, 0.0, depth);

    glEnd();
}



FBO_Filter::FBO_Filter( CGprofile cgp,      ///< Desired profile.  Typically
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
                        char **args )
{
    depth = -1.0;
    //Load the Cg Program.
    cgProfile = cgp;
    cgContext = cgCreateContext();
    errContext = cgContext;

    cgSetErrorCallback(cgErrorCallback);
    if ( name != NULL )  {
        cgProgram  = load_cgprogram(cgProfile, name, args );
        cgGLLoadProgram( cgProgram );
    }
    else {
        cgProgram = 0;
    }

    tWidth = W;
    tHeight = H;

    //create the framebuffer object.
    oTex = outputTex ;
    glGenFramebuffersEXT( 1, &fb );
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, fb );
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, oTex );
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT,
                              GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, oTex, 0);
    CHECK_FRAMEBUFFER_STATUS()
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}



///TODO figure out readback formats from target texture information.
GLuint FBO_Filter::apply( GLuint iTex,    ///< OpenGL texture object to use as input
                          bool FBOtex,     ///< set to true if the input texture is the result of a previous
                          ///  FBO_filter.  False if not.
                          GLvoid *rb_buf ,
                          GLenum rb_fmt ,
                          GLenum rb_type  )
{
    GLint activeARB;
    glGetIntegerv( GL_CLIENT_ACTIVE_TEXTURE_ARB, &activeARB );
    glActiveTextureARB( GL_TEXTURE0_ARB );

    //XXX all this frambuffer binding is suboptimal, but will work for now.
    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, fb );
    glBindTexture(GL_TEXTURE_RECTANGLE_NV, iTex );
    glEnable(GL_TEXTURE_RECTANGLE_NV);

    renderBegin();
    cgGLEnableProfile(cgProfile);
    cgGLBindProgram(cgProgram);
    ( FBOtex ? drawQuadFBT() : drawQuadTex() );
    cgGLDisableProfile(cgProfile);
    renderEnd();

    if ( rb_buf != NULL ) {
        glReadPixels( 0, 0, tWidth, tHeight, rb_fmt, rb_type, rb_buf );
    }

    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0);

    glActiveTextureARB( activeARB );
    return oTex ;
}




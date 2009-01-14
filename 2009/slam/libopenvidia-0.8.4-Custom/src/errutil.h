/*
#define ERRCHECK() \
{\
\
GLenum errCode;\
const GLubyte *errString;\
  if ((errCode = glGetError()) != GL_NO_ERROR) {\
    errString = gluErrorString(errCode);\
    fprintf (stderr, "OpenGL Error: %s at %s:%d\n", errString,  __FILE__,__LINE__);\
    exit(1);\
  }\
}\
*/
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

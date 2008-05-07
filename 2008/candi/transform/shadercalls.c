
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <sys/time.h>
#include <GL/glx.h>
#if __cplusplus
extern "C"{
#endif



//translation of following type definition:
//my_glGenProgramsARB is pointer to function taking (GLuint, GLuint*) that returns void
static void (*my_glGenProgramsARB)(GLuint, GLuint *) = NULL;
void (*my_glBindProgramARB)(GLuint, GLuint) = NULL;
static void (*my_glProgramStringARB)(GLuint, GLuint, GLint, const GLbyte *) = NULL;

static void (*my_glActiveTextureARB)(GLenum) = NULL;
static void (*my_glMultiTexCoord3fARB)(GLenum, GLfloat, GLfloat, GLfloat) = NULL;


static void *
get_proc_address(const char *name)
{
#ifdef _WIN32
	return (void *)wglGetProcAddress(name);
#else
	return (void *)glXGetProcAddress((const GLubyte *)name);
#endif /* _WIN32 */
}

static void
set_function_pointers(void)
{
	my_glGenProgramsARB = (void (*)(GLuint, GLuint *))get_proc_address("glGenProgramsARB");
	if(!my_glGenProgramsARB) {
		fprintf(stderr, "set_function_pointers(): glGenProgramsARB failed\n");
		exit(1);
	}
	my_glBindProgramARB = (void (*)(GLuint, GLuint))get_proc_address("glBindProgramARB");
	if(!my_glBindProgramARB) {
		fprintf(stderr, "set_function_pointers(): glBindProgramARB failed\n");
		exit(1);
	}
	my_glProgramStringARB = (void (*)(GLuint, GLuint, GLint, const GLbyte *))get_proc_address("glProgramStringARB");
	if(!my_glProgramStringARB) {
		fprintf(stderr, "set_function_pointers(): glProgramStringARB failed\n");
		exit(1);
	}
	my_glActiveTextureARB = (void (*)(GLuint))get_proc_address("glActiveTexture");
	if(!my_glActiveTextureARB) {
		fprintf(stderr, "set_function_pointers(): glActiveTextureARB failed\n");
		exit(1);
	}
	my_glMultiTexCoord3fARB = (void (*)(GLuint, GLfloat, GLfloat, GLfloat))get_proc_address("glMultiTexCoord3fARB");
	if(!my_glMultiTexCoord3fARB) {
		fprintf(stderr, "set_function_pointers(): glMultiTexCoord3fARB failed\n");
		exit(1);
	}
}

/*
 * this function returns a string containing
 * the contents of the specified shader file
 */
static char *
load_program_string(const char *filename)
{
	static char program_string[16384];
	FILE *fp;
	unsigned int len;

	fp = fopen(filename, "r");
	if(!fp){
		printf("ERROR: Opening shader file failed\n");
		exit(1);
		return NULL;
		}

	len = fread(program_string, 1, 16384, fp);
	program_string[len] = '\0';
	fclose(fp);

	return program_string;
}

/*
 * this function loads the shader from the specified
 * file and returns a shader number that can be passed
 * to ARB_fragment_program functions
 */
static unsigned int
load_shader(GLuint type, const char *filename)
{
	unsigned int shader_num;
	char *program_string;

	program_string = load_program_string(filename);

	glEnable(type);
	my_glGenProgramsARB(1, &shader_num);
	my_glBindProgramARB(type, shader_num);
	my_glProgramStringARB(type, GL_PROGRAM_FORMAT_ASCII_ARB, strlen(program_string), (const GLbyte *)program_string);
	glDisable(type);

	return shader_num;
}
unsigned int shader_num;
void
shader_init(void)
{
	set_function_pointers();
	shader_num = load_shader(GL_FRAGMENT_PROGRAM_ARB, "transform/shader.pso");
}
#if __cplusplus
}
#endif

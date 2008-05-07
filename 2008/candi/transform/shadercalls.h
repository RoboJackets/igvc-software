#ifndef SHADERCALLS
#define SHADERCALLS

#if __cplusplus
extern "C"{
#endif



void shader_init(void);
extern void (*my_glBindProgramARB)(GLuint, GLuint);
extern unsigned int shader_num;







#if __cplusplus
}
#endif

#endif

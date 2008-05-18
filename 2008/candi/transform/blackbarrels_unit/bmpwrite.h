#ifndef bmpwrite_h
#define bmpwrite_h

#if __cplusplus
extern "C"{
#endif
int 
write_bmp(const char *filename, int width, int height, char *rgb);
#if __cplusplus
}
#endif

#endif

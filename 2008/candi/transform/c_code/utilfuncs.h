#ifndef utilfunc_h
#define utilfunc_h

#include "image.h"

int ImageLoad(char *filename, Image *image);

void LoadGLTextures();
void drawRose(void);
inline unsigned long long nanotime(void);
long long currentTimeMillis();
long long currentTimeMicros();
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)
#endif

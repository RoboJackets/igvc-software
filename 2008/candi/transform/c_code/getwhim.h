#ifndef getwhim_h
#define getwhim_h



/* --- Public C Functions --- */
#if __cplusplus
extern "C" {
#endif
char * getwhim(Image* im);

//member variable
Buffer2D<PixelRGB> whim;

#if __cplusplus
}
#endif

#endif

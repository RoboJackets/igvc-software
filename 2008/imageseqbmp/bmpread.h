#ifndef BMPREAD_H
#define BMPREAD_H

#if __cplusplus
extern "C" {
#endif

typedef struct Image_struct {
    unsigned long sizeX;
    unsigned long sizeY;
    unsigned char *data;
} Image;

int ImageLoad(char *filename, Image *image);
void ImageFree(Image *image);

#if __cplusplus
}
#endif

#endif

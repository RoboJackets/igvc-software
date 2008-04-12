#include "imageutils.h"


/**
 * Creates a new Buffer2D<PixelRGB> that is a <i>view</i>
 * of the specified <tt>img</tt>. In particular, the pixel
 * data array is shared.
 */

Buffer2D<PixelRGB>* imageAsBuffer2D (Image* img) {
	Buffer2D<PixelRGB>* buf = new Buffer2D<PixelRGB>(); //Creates new buffer as alias for the input, be carefull to free when done
	buf->width = img->width;
	buf->height = img->height;
	buf->data = (PixelRGB*) img->data;
	return buf;
}

Buffer2D<PixelRGB> imageAsStaticBuffer2D (Image* img) {
	Buffer2D<PixelRGB> buf = Buffer2D<PixelRGB>();
	buf.width = img->width;
	buf.height = img->height;
	buf.data = (PixelRGB*) img->data;
	return buf;
}

void booltoRGB (Buffer2D<bool>& img, Buffer2D<PixelRGB>& dst){
	int ii;
	int buffLength = dst.width * dst.height;

	/*standard reverse boolean conversion*/
	for(ii = 0; ii < buffLength; ii++){
		if(img[ii])
			dst[ii].r = dst[ii].g = dst[ii].b = 255;
		else
			dst[ii].r = dst[ii].g = dst[ii].b = 0;
	}	
}

void RGBtoBool (Buffer2D<PixelRGB>& img, Buffer2D<bool>& dst) {
	int ii;
	int buffLength = img.width * img.height;

	/*standard boolean conversion*/
	for(ii = 0; ii < buffLength; ii++){
		if(img[ii].r > 0 || img[ii].g > 0 || img[ii].b > 0)
			dst[ii] = 1;
		else
			dst[ii] = 0;
	}
}

//Does an inverse mask operation on the rgb image
void blackout (Buffer2D<bool>& mask, Buffer2D<PixelRGB>& img) {
	int ii;
	int buffLength = img.numElements();

	for(ii = 0; ii < buffLength; ii++){
		if(mask[ii]){
			img[ii].r = img[ii].g = img[ii].b = 0;
		}
	}	
}

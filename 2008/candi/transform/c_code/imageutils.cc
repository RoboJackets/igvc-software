#include "imageutils.h"


/**
 * Creates a new Buffer2D<PixelRGB> that is a <i>view</i>
 * of the specified <tt>img</tt>. In particular, the pixel
 * data array is shared.
 */

Buffer2D<PixelRGB>* imageAsBuffer2D(Image* img) {
	Buffer2D<PixelRGB>* buf = new Buffer2D<PixelRGB>(); //Creates new buffer as alias for the input, be carefull to free when done
	buf->width = img->width;
	buf->height = img->height;
	buf->data = img->data; 
	return buf;
}

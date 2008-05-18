#include "PCamera.h"
#include "transform/pglobals_extern.h"
#include "transform/blackbarrels_unit/bmpwrite.h"
#include <GL/gl.h>

/*static*/ PCamera PCamera::INSTANCE;

QSize PCamera::getSize() const {
	return QSize(frame.width, frame.height);
}

Pixel* PCamera::getRGB() {
	return frame.data;
}

void PCamera::update() {
	frame.resize(TRANSFORM_OUTPUT_WIDTH, TRANSFORM_OUTPUT_HEIGHT);
	glReadPixels(	0						,	//GLint x,
			     	0						,	//GLint y,
			     	TRANSFORM_OUTPUT_WIDTH	,	//GLsizei width,
			     	TRANSFORM_OUTPUT_HEIGHT	,	//GLsizei height,
			     	GL_BGRA					,	//GLenum format,
			     	GL_UNSIGNED_BYTE		,	//GLenum type,
			     	frame.data	);				//GLvoid *pixels
}

void PCamera::writeFrameToDisk() {
	frameToWrite.resize(TRANSFORM_OUTPUT_WIDTH, TRANSFORM_OUTPUT_HEIGHT);
	glReadPixels(	0						,	//GLint x,
			     	0						,	//GLint y,
			     	TRANSFORM_OUTPUT_WIDTH	,	//GLsizei width,
			     	TRANSFORM_OUTPUT_HEIGHT	,	//GLsizei height,
			     	GL_RGB					,	//GLenum format,
			     	GL_UNSIGNED_BYTE		,	//GLenum type,
			     	frameToWrite.data	);		//GLvoid *pixels
			     	
	//write_bmp(const char *filename, int width, int height, char *rgb);
}

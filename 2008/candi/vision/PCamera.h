#ifndef _P_CAMERA_H_
#define _P_CAMERA_H_

#include "Camera.h"
#include "Pixel.h"
#include "Buffer2D.h"
#include "transform/blackbarrels_unit/PixelRGB.h"
#include <qstring.h>

class PCamera: public Camera
{
public:
	static PCamera INSTANCE;
	
private:
	PCamera() {}
	~PCamera() {}

public:
	QString getName() const { return "Paul Camera";}
	QSize getSize() const;

public:
	bool isValid() const { return true; }
	Pixel *getRGB();

	void update();
	void writeFrameToDisk();

protected:
	Buffer2D<Pixel> frame;
	Buffer2D<blackbarrels::PixelRGB> frameToWrite;
};
#endif // _P_CAMERA_H_

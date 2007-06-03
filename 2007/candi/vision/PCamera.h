#ifndef _P_CAMERA_H_
#define _P_CAMERA_H_

#include "Camera.h"
#include "Pixel.h"
#include "Buffer2D.h"
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

protected:
	Buffer2D<Pixel> frame;
};
#endif // _P_CAMERA_H_

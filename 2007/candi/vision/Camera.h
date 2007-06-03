#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "Pixel.h"
#include "types.h"
#include <QSize>
#include <QList>

// Camera objects automatically add themselves to a global list, which
// the GUI code uses to populate the image source combobox.
// This works the same way as VideoViews, so see ../gui/VideoView.h for
// details.

// Each type of Camera declares a static field INSTANCE containing the
// default instance of the camera type.

// <abstract>
class Camera
{
public:
	Camera();
	virtual ~Camera();

	virtual QString getName() const = 0;
	virtual QSize getSize() const = 0;

	// Returns true if this camera is capable of providing images.
	virtual bool isValid() const = 0;

	// Returns a pointer to 32-bit RGB data, in BGR0 format
	// Pixels are stored in the following format:
	/*
	 B G R 0 B G R 0 B G R 0
	 B G R 0 B G R 0 B G R 0
	 B G R 0 B G R 0 B G R 0
	 */
	// NOTE: Invokers of this method must be careful to call
	//       unlock() when done with the returned image buffer.
	virtual Pixel *getRGB();

	// Gets the next frame from the camera.  May block.
	virtual void update();

	// Call this when you have finished processing the last frame's data
	// (after invoking getRGB).
	// (Some camera types need to be told when the DMA buffer can be released.)
	virtual void unlock();
	
	// Current image source for vision processing
	static Camera* current; // = 0;

	// Returns a list of all cameras.
	static const QList<Camera *> *getCameraList() { return cameraList; }

private:
	static QList<Camera *> *cameraList;
};

#endif // _CAMERA_H_


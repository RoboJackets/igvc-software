#include "Camera.h"
#include "Pixel.h"

/*static*/ Camera* Camera::current;
/*static*/ QList<Camera *> *Camera::cameraList = 0;

Camera::Camera()
{
	if (!cameraList)
	{
		cameraList = new QList<Camera *>;
	}

	cameraList->append(this);
}

Camera::~Camera()
{
	cameraList->removeAll(this);
}

Pixel *Camera::getRGB()
{
	return 0;
}

void Camera::update()
{
}

void Camera::unlock()
{
}


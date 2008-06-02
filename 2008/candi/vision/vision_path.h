#ifndef _VISION_PATH_H_
#define _VISION_PATH_H_

#include "Buffer2D.h"
#include "Pixel.h"
#include "Point2D.h"
#include "Graphics.h"

extern void visGenPath();
extern Buffer2D<bool> visPathView;

Point2D<int> robotWidthScan();
void visPathControlMotors(Point2D<int> goal);

#define ROBOT_WIDTH 30 //pixels wide


#endif

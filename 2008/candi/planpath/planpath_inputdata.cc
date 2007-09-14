#include "planpath_inputdata.h"
#include "vision/vision_color.h"	// for 'pixelIsWhite', 'pixelIsOrange'

QVector<Obstacle> obstacles;

/*void planpath_importObstacles() {
	int width = pixelIsWhite.width;
	int height = pixelIsWhite.height;	
	
	obstacles.clear();
	for (int y=0, off=0; y<height; y++) {
		for (int x=0; x<width; x++, off++) {
			ObstacleType obstacleType = OT_NONE;
			if (pixelIsOrangle[off]) obstacleType = OT_BARREL;
			if (pixelIsWhite[off]) obstacleType = OT_LINE;
			
			if (obstacleType != OT_NONE) {
				obstacles.append(Obstacle(
					x-(width/2), (height-1)-y,		// convert pixel coordinates to robot coordinates
					obstacleType));
			}
		}
	}
}*/

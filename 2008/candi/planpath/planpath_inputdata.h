#ifndef PLANPATH_INPUTDATA
#define PLANPATH_INPUTDATA

#include "QVector"

typedef enum {
	OT_NONE		= 0,	// special
	OT_LINE		= 1,
	OT_BARREL	= 2,
} ObstacleType;

class Obstacle {
	double x;			// (-) = left, (0) = center, (+) = right
	double y;			// (-) = behind, (0) = side, (+) = front
	ObstacleType type;
};

// INPUT
extern QVector<Obstacle> obstacles;

/** Imports the set of obstacles in view of the camera. */
void planpath_importObstacles();

#endif

/*
 * Map.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Matthew Barulic
 */

#ifndef MAP_H_
#define MAP_H_

#include "../common/Pose.hpp"
#include "MapPoint.hpp"

namespace IGVC {
namespace Mapping {

class Map {
public:
	Map();
	~Map();
	bool poseIsOpen(Pose p);
	void setPoint(float x, float y, ObjectType t);
private:
	std::list<MapPoint> points;
};

} /* namespace Mapping */
} /* namespace IGVC */
#endif /* MAP_H_ */

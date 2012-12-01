/*
 * Map.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Matthew Barulic
 */

#ifndef MAP_H_
#define MAP_H_

#include "../common/Pose.hpp"

namespace IGVC {
namespace Mapping {

class Map {
public:
	Map();
	~Map();
private:
	std::list<MapPoint> points;
	bool poseIsOpen(Pose p);
};

} /* namespace Mapping */
} /* namespace IGVC */
#endif /* MAP_H_ */

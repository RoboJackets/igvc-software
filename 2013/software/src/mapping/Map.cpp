/*
 * Map.cpp
 *
 *  Created on: Nov 30, 2012
 *      Author: Matthew Barulic
 */

#include "Map.h"

namespace IGVC {
namespace Mapping {

Map::Map() {
	// TODO Auto-generated constructor stub

}

bool Map::poseIsOpen(Pose p) {
	return false;
}

void Map::setPoint(float x, float y, ObjectType t) {
	MapPoint mp;
	mp.x = x;
	mp.y = y;
	mp.type = t;
	points.push_back(mp);
}

Map::~Map() {
	// TODO Auto-generated destructor stub
}

} /* namespace Mapping */
} /* namespace IGVC */

/*
 * MapPoint.hpp
 *
 *  Created on: Nov 30, 2012
 *      Author: Matthew Barulic
 */

#ifndef MAPPOINT_HPP_
#define MAPPOINT_HPP_

namespace IGVC {
namespace Mapping {

enum ObjectType {
	Line,
	Barrel,
	Flag,
	Fence,
	Unknown
};

class MapPoint {
public:
	float x;
	float y;
	ObjectType type;
};

} //Mapping
} //IGVC


#endif /* MAPPOINT_HPP_ */

#ifndef MAPGEN_H
#define MAPGEN_H


/*
 * This file contains the robot's SLAM processing code.
 *   by: Chris McClanahan
 *
 */

class MapGen
{
public:
	MapGen();
	virtual ~MapGen();
public:
    void genMap();
	void getFeatures();

};

#endif // MAPGEN_H

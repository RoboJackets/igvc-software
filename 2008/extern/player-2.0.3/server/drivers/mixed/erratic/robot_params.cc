/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *                   
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include "robot_params.h"


RobotParams_t erratic_params = 
{
	0.001534,
	"Videre",
	0.011,
	0.780, //This is the empirically correct value, but doesn't match wheel size calculation
	0,
	1.626,
	1,
	1,
	0,
	0,
	0,
	"",
	"",
	0,
	1,
	0,
	0,
	0,
	300,
	1000,
	0,
	5,
	5,
	1,
	0,
	0,
	0,
	120,
	392, // length
	180,
	415, // width
	61, // axle distance to center of robot (positive forward)
	0,
	0,
	0,
	1,
	1,
	8,
	"erratic",
	38400,
	0,
	0,
	0,
	0,
	20,
	1.20482,
	{
		{ 73, 105, 90 },
		{ 130, 78, 41 },
		{ 154, 30, 15 },
		{ 154, -30, -15 },
		{ 130, -78, -41 },
		{ 73, -105, -90 },
		{ -146, -60, -145 },
		{ -146, 60, 145 },
	},
	2,
	{
		{0.1, 0.1, 0},
		{0.1, -0.1, 0},
	}
};


RobotParams_t *RobotParams[1] = {&erratic_params};

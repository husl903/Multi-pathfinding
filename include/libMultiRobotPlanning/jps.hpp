#ifndef WARTHOG_JPS_H
#define WARTHOG_JPS_H

// jps.h
//
// This file contains the namespace for common definitions
// required by the various classes that use Jump Points.
// Note that the operations defined here assume corner
// cutting is not allowed. This change requires some slight 
// modification to the basic Jump Point Search method. 
// For details see:
// [D Harabor and A Grastien, The JPS+ Pathfinding System, SoCS, 2012]
//
// @author: dharabor
// @created: 04/09/2012
//


#include <stdint.h>
#include <unordered_map>

namespace libMultiRobotPlanning
{

namespace jps{
typedef enum
{
	NONE = 0,
	NORTH = 1,
	SOUTH = 2,
	EAST = 4,
	WEST = 8,
	NORTHEAST = 16,
	NORTHWEST = 32, 
	SOUTHEAST = 64,
	SOUTHWEST = 128,
    ALL = 255
} direction;
}
}

#endif


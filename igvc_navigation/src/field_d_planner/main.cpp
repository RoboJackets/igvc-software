/**
Solves for an optimal path using the Field D* incremental search algorithm.

Field D* implementation details can be found in FieldDPlanner.h

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: January 23rd, 2018
*/

#include "FieldDPlanner.h"
#include "Graph.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "field_d_planner");
  ros::NodeHandle nh;
  FieldDPlanner field_d_planner(&nh);
  return 0;
}

#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "pathplanner.hpp"
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"

class AStarPlanner : public PathPlanner
{
public:
    AStarPlanner();

    path_t GetPath();

protected:
    /** Runs AStar on the given search problem. The 'path' variable will be updated with the resulting path. */
    void run();
    /**
     * @brief Checks if the path is still valid with the current searchproblem
     * @return True if the path is collision free, the goal has not changed, and the robot is still on the path.
     */
    bool pathIsValid();

    float distanceFromPath(SearchLocation point);

    IGVCSearchProblem searchproblem;
    path_t path;
    /** Flags used to ensure that all 3 pieces have been set before the algorithm runs. */
    bool mapSet;
    bool startSet;
    bool goalSet;

public slots:
    void OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map);
    void OnNewStartPos(RobotPosition pos);
    void OnNewGoalPos(RobotPosition pos);
};

#endif // ASTARPLANNER_H

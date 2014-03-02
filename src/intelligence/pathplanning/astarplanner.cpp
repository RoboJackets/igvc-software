#include "astarplanner.h"
#include <common/config/configmanager.h>

AStarPlanner::AStarPlanner()
{
    ConfigManager::Instance().getValue("AStarPlanner", "GoalThreshold", 2.0);
}

path_t AStarPlanner::GetPath()
{
    return path;
}

void AStarPlanner::run()
{
    auto newpath = GraphSearch::AStar(searchproblem);
    path.clear();
    for(int i = 0; i < newpath.getNumberOfSteps(); i++)
        path.push_back(std::pair<SearchMove,SearchLocation>(newpath.getAction(i), newpath.getState(i)));
    OnNewPath(path);
}

bool AStarPlanner::pathIsValid()
{
    if(path[path.size()-1].second.distTo(searchproblem.Goal) > ConfigManager::Instance().getValue("AStarPlanner", "GoalThreshold", 2.0))
        return false;

    return false;
}

void AStarPlanner::OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map)
{
    searchproblem.Map = *map;
    if(!pathIsValid())
        run();
}

void AStarPlanner::OnNewStartPos(RobotPosition pos)
{
    searchproblem.Start = SearchLocation(pos.X, pos.Y, pos.Heading);
    if(!pathIsValid())
        run();
}

void AStarPlanner::OnNewGoalPos(RobotPosition pos)
{
    searchproblem.Goal = SearchLocation(pos.X, pos.Y, pos.Heading);
    if(!pathIsValid())
        run();
}

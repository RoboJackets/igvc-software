#include "astarplanner.h"
#include <common/config/configmanager.h>
#include <pcl/kdtree/kdtree_flann.h>

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
    if(distanceFromPath(searchproblem.Start) > ConfigManager::Instance().getValue("AStarPlanner", "OnPathThreshold", 0.6))
        return false;
    // TODO : check for collisions
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(searchproblem.Map.makeShared());
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = ConfigManager::Instance().getValue("AStarPLanner", "CollisionThreshold", 1.0);
    for(auto pair : path)
    {
        pcl::PointXYZ point;
        point.x = pair[1].X;
        point.y = pair[1].Y;
        point.z = 0;
        if( kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            return false;
        }
    }
    return true;
}

float AStarPlanner::distanceFromPath(SearchLocation point)
{
    float minDist = point.distTo(searchproblem.Start);
    for(auto pair : path)
    {
        float dist = point.distTo(pair.second);
        if(dist < minDist)
            minDist = dist;
    }
    return minDist;
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

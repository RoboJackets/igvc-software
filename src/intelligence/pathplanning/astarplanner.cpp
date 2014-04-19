#include "astarplanner.h"
#include <common/config/configmanager.h>
#include <pcl/kdtree/kdtree_flann.h>

AStarPlanner::AStarPlanner()
{
    /*
     * These ConfigManager calls just make sure the config values get added
     * to the file even if the algorithm doesn't run. This is simply for
     * ease of use, ensuring that the config values appear in the editor at
     * the start of the application.
     */
    ConfigManager::Instance().getValue("AStarPlanner", "GoalThreshold", 2.0);
    ConfigManager::Instance().getValue("AStarPlanner", "OnPathThreshold", 0.6);
    ConfigManager::Instance().getValue("AStarPLanner", "CollisionThreshold", 1.0);
    ConfigManager::Instance().getValue("AStarPlanner", "Velocity", 1.0);
    ConfigManager::Instance().getValue("AStarPlanner", "DeltaT", 1.0);
    ConfigManager::Instance().getValue("Robot", "Baseline", 0);
    ConfigManager::Instance().getValue("AStarPlanner", "CollisionRadius", 1.0);

    mapSet = false;
    goalSet = false;
    startSet = false;
}

path_t AStarPlanner::GetPath()
{
    return path;
}

void AStarPlanner::run()
{
    if(mapSet && startSet && goalSet)
    {
        searchproblem.Speed = ConfigManager::Instance().getValue("AStarPlanner", "Velocity", 1.0);
        searchproblem.DeltaT = ConfigManager::Instance().getValue("AStarPlanner", "DeltaT", 1.0);
        searchproblem.PointTurnsEnabled = false;
        searchproblem.Baseline = ConfigManager::Instance().getValue("Robot", "Baseline", 0);
        searchproblem.Threshold = ConfigManager::Instance().getValue("AStarPlanner", "CollisionRadius", 1.0);
        searchproblem.GoalThreshold = ConfigManager::Instance().getValue("AStarPlanner", "GoalThreshold", 1.0);
        auto newpath = GraphSearch::AStar(searchproblem);
        path.clear();
        for(int i = 0; i < newpath.getNumberOfSteps(); i++)
            path.push_back(std::pair<SearchMove,SearchLocation>(newpath.getAction(i), newpath.getState(i+1)));
        OnNewPath(path);
    }
}

bool AStarPlanner::pathIsValid()
{
    if(path.size() == 0)
        return false;
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
        point.x = pair.second.x;
        point.y = pair.second.y;
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
    mapSet = true;
    if(!pathIsValid())
        run();
}

void AStarPlanner::OnNewStartPos(RobotPosition pos)
{
    searchproblem.Start = SearchLocation(pos.X, pos.Y, pos.Heading);
    startSet = true;
    if(!pathIsValid())
        run();
}

void AStarPlanner::OnNewGoalPos(RobotPosition pos)
{
    searchproblem.Goal = SearchLocation(pos.X, pos.Y, pos.Heading);
    goalSet = true;
    if(!pathIsValid())
        run();
}

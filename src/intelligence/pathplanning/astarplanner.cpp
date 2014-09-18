#include "astarplanner.h"
#include <common/config/configmanager.h>
#include <common/logger/logger.h>
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
    ConfigManager::Instance().getValue("AStarPlanner", "CollisionThreshold", 1.0);
    ConfigManager::Instance().getValue("AStarPlanner", "Velocity", 1.0);
    ConfigManager::Instance().getValue("AStarPlanner", "DeltaT", 1.0);
    ConfigManager::Instance().getValue("Robot", "Baseline", 0);
    ConfigManager::Instance().getValue("AStarPlanner", "CollisionRadius", 1.0);

    mapSet = false;
    goalSet = false;
    startSet = false;

    replanRequested = false;
    thread = boost::thread(boost::bind(&AStarPlanner::run, this));
}

AStarPlanner::~AStarPlanner()
{

}

path_t AStarPlanner::GetPath()
{
    return path;
}

void AStarPlanner::run()
{
    while(true)
    {
        if(mapSet && startSet && goalSet && replanRequested)
        {
            searchproblem.Speed = ConfigManager::Instance().getValue("AStarPlanner", "Velocity", 1.0);
            searchproblem.DeltaT = ConfigManager::Instance().getValue("AStarPlanner", "DeltaT", 1.0);
            searchproblem.PointTurnsEnabled = false;
            searchproblem.Baseline = ConfigManager::Instance().getValue("Robot", "Baseline", 1.0);
            searchproblem.Threshold = ConfigManager::Instance().getValue("AStarPlanner", "CollisionRadius", 1.0);
            searchproblem.GoalThreshold = ConfigManager::Instance().getValue("AStarPlanner", "GoalThreshold", 1.0);
            auto newpath = GraphSearch::AStar(searchproblem);
            path.clear();
            for(int i = 0; i < newpath.getNumberOfSteps(); i++)
                path.push_back(std::pair<SearchMove,SearchLocation>(newpath.getAction(i), newpath.getState(i+1)));
            OnNewPath(path);
            try {
                boost::this_thread::interruption_point();
            } catch(...) {
                return;
            }
        }
        usleep(300000);
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
    if(searchproblem.Map.empty())
        return true;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(searchproblem.Map.makeShared());
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = ConfigManager::Instance().getValue("AStarPlanner", "CollisionThreshold", 1.0);
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
        replanRequested = true;
}

void AStarPlanner::OnNewStartPos(RobotPosition pos)
{
    searchproblem.Start = SearchLocation(pos.X, pos.Y, pos.Heading);
    startSet = true;
    if(!pathIsValid())
        replanRequested = true;
}

void AStarPlanner::OnNewGoalPos(RobotPosition pos)
{
    searchproblem.Goal = SearchLocation(pos.X, pos.Y, pos.Heading);
    goalSet = true;
    std::stringstream msg;
    msg << "Path planner received new goal pos " << pos;
    Logger::Log(LogLevel::Debug, msg.str());
    if(!pathIsValid())
        replanRequested = true;
}

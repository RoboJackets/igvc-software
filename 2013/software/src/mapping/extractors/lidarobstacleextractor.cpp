#include "lidarobstacleextractor.h"
#include <algorithm>
#include <iostream>

namespace IGVC
{
namespace mapping
{
namespace extractors
{

LidarObstacleExtractor::LidarObstacleExtractor(Lidar *device)
    : LonNewLidarData(this)
{
    if(device)
    {
        device->onNewData += &LonNewLidarData;
        _device = device;
    }
    jumpDistThreshold = 0.1;
    linearityThreshold = 0.475;
    sizeThreshold = 10;
}

void LidarObstacleExtractor::disconnect()
{
    if(_device)
    {
        _device->onNewData -= &LonNewLidarData;
        _device = 0;
    }
}

void LidarObstacleExtractor::onNewLidarData(LidarState data)
{
    using namespace std;

    vector<Obstacle *> obstacles;

    vector<Obstacle*> filtered = filterCircularObstacles(filterLinearObstacles(filterBySize(clusterByJumps(clusterByValidity(&data)))));
    obstacles.insert(obstacles.end(), filtered.begin(), filtered.end());

    onNewData(obstacles);
}

std::vector<PointArrayObstacle*> LidarObstacleExtractor::clusterByValidity(LidarState *data)
{
    using namespace std;

    vector<PointArrayObstacle*> clusters;

    bool inCluster = false;

    PointArrayObstacle *cluster;

    for(int i = 0; i < 1024; i++)
    {
        LidarPoint &pt = data->points[i];

        float x = cos(pt.angle)*pt.distance;
        float y = sin(pt.angle)*pt.distance;

        if(pt.valid)
        {
            if(!inCluster)
            {
                clusters.push_back(new PointArrayObstacle());
                cluster = clusters.at(clusters.size()-1);
                cluster->addPoint(Vector2f(x,y));
                inCluster = true;
            } else {
                cluster->addPoint(Vector2f(x,y));
            }
        } else {
            if(inCluster)
            {
                inCluster = false;
            }
        }
    }

    return clusters;
}

std::vector<PointArrayObstacle*> LidarObstacleExtractor::clusterByJumps(std::vector<PointArrayObstacle*> data)
{
    using namespace std;
    vector<PointArrayObstacle*> clusters;

    bool inCluster = false;

    PointArrayObstacle *newcluster;

    for(vector<PointArrayObstacle*>::iterator iter = data.begin(); iter < data.end(); iter++)
    {
        PointArrayObstacle* cluster = (*iter);
        Vector2f* points = cluster->getPoints();
        for(int i = 0; i < cluster->getNumPoints()-1; i++)
        {
            if(!inCluster)
            {
                clusters.push_back(new PointArrayObstacle());
                newcluster = clusters.at(clusters.size()-1);
                newcluster->addPoint(points[i]);
                inCluster = true;
            } else {
                //float dist = points[i].distanceTo(points[i+1]);
                float dist = (points[i+1] - points[i]).norm();
                newcluster->addPoint(points[i]);
                if(dist >= jumpDistThreshold)
                {
                    // Jump detected, break cluster
                    inCluster = false;
                }
            }
        }
        if(inCluster)
            newcluster->addPoint(points[cluster->getNumPoints()-1]);
        inCluster = false;
    }
    return clusters;
}

std::vector<PointArrayObstacle*> LidarObstacleExtractor::filterBySize(std::vector<PointArrayObstacle*> data)
{
    using namespace std;
    vector<PointArrayObstacle*> clusters;

    for(vector<PointArrayObstacle*>::iterator iter = data.begin(); iter < data.end(); iter++)
    {
        PointArrayObstacle* cluster = (*iter);
        if(cluster->getNumPoints() > sizeThreshold)
        {
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

Vector2f LidarObstacleExtractor::calculateCenterOfMass(Vector2f *points, int numPoints)
{
    Vector2f cm;
    for(int i = 0; i < numPoints; i++)
    {
        cm += points[i];
    }
    cm /= numPoints;
    return cm;
}

float LidarObstacleExtractor::calculateMoment(Vector2f *points, int numPoints, int p, int q)
{
    Vector2f cm = calculateCenterOfMass(points, numPoints);
    float u = 0;
    for(int i = 0; i < numPoints; i++)
    {
        Vector2f &point = points[i];
        u += pow((point[0] - cm[0]), p) * pow((point[1] - cm[1]),q);
    }
    u /= numPoints;
    return u;
}

float LidarObstacleExtractor::calculateAngleOfOrientation(Vector2f *points, int numPoints)
{
    return 0.5 * atan( ( 2.0 * calculateMoment(points, numPoints, 1, 1) ) / ( calculateMoment(points, numPoints, 2, 0) - calculateMoment(points, numPoints, 0, 2) ) );
}

float LidarObstacleExtractor::measureLinearity(Vector2f *points, int numPoints)
{
    int k = numPoints / 2;
//    Point cm = calculateCenterOfMass(points, numPoints);
    float angle = calculateAngleOfOrientation(points, numPoints);
    float M = tan(angle);
//    Point normal(-M,1);
    Vector2f localNormals[k];
    for(int i = 0; i < k; i++)
    {
        Vector2f a = points[rand() % numPoints];
        Vector2f b;
        while(b == a)
        {
            b = points[rand() % numPoints];
        }
        float m = (b[1] - a[1]) / (b[0] - a[0]);
        float norm = sqrt(m*m + 1);
        float dp = m*M + 1;
        if(dp < 0)
        {
            localNormals[i] = Vector2f(m/norm, -1/norm);
        } else {
            localNormals[i] = Vector2f(-m/norm, 1/norm);
        }
    }
    Vector2f normalToOrientation(0,0);
    for(int i = 0; i < k; i++)
    {
        normalToOrientation += localNormals[i];
    }

    normalToOrientation /= numPoints;

    return normalToOrientation.norm();
}

std::vector<Obstacle*> LidarObstacleExtractor::filterLinearObstacles(std::vector<PointArrayObstacle*> data)
{
    using namespace std;

    std::vector<Obstacle*> obstacles;

    for(vector<PointArrayObstacle*>::iterator iter = data.begin(); iter < data.end(); iter++)
    {
        PointArrayObstacle* cluster = (*iter);
        float linearity = measureLinearity(cluster->getPoints(), cluster->getNumPoints());
        if(linearity >= linearityThreshold)
        {
            // This is a line
            obstacles.push_back(new LinearObstacle(cluster->getPoints()[0], cluster->getPoints()[cluster->getNumPoints()-1]));
        } else {
            // This is not a line
            obstacles.push_back(cluster);
        }
    }

    return obstacles;
}

std::vector<Obstacle*> LidarObstacleExtractor::filterCircularObstacles(std::vector<Obstacle*> data)
{
    using namespace std;
    vector<Obstacle*> filtered;

    for(vector<Obstacle*>::iterator iter = data.begin(); iter != data.end(); iter++)
    {
        Obstacle* obst = (*iter);
        PointArrayObstacle* raw = dynamic_cast<PointArrayObstacle*>(obst);
        if(raw != 0)
        {

        } else {
            filtered.push_back(obst);
        }
    }

    return filtered;
}

}
}
}

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
    }
    jumpDistThreshold = 0.1;
    linearityThreshold = 0.475;
}

void LidarObstacleExtractor::onNewLidarData(LidarState data)
{
    using namespace std;

    vector<Obstacle *> obstacles;

    vector<Obstacle*> filtered = filterLinearObstacles(clusterByJumps(clusterByValidity(&data)));
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
                cluster->addPoint(Point(x,y));
                inCluster = true;
            } else {
                cluster->addPoint(Point(x,y));
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
        Point* points = cluster->getPoints();
        for(int i = 0; i < cluster->getNumPoints()-1; i++)
        {
            if(!inCluster)
            {
                clusters.push_back(new PointArrayObstacle());
                newcluster = clusters.at(clusters.size()-1);
                newcluster->addPoint(points[i]);
                inCluster = true;
            } else {
                float dist = points[i].distanceTo(points[i+1]);
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

Point LidarObstacleExtractor::calculateCenterOfMass(Point *points, int numPoints)
{
    Point cm;
    for(int i = 0; i < numPoints; i++)
    {
        cm.x += points[i].x;
        cm.y += points[i].y;
    }
    cm.x /= numPoints;
    cm.y /= numPoints;
    return cm;
}

float LidarObstacleExtractor::calculateMoment(Point *points, int numPoints, int p, int q)
{
    Point cm = calculateCenterOfMass(points, numPoints);
    float u = 0;
    for(int i = 0; i < numPoints; i++)
    {
        Point &point = points[i];
        u += pow((point.x - cm.x), p) * pow((point.y - cm.y),q);
    }
    u /= numPoints;
    return u;
}

float LidarObstacleExtractor::calculateAngleOfOrientation(Point *points, int numPoints)
{
    return 0.5 * atan( ( 2.0 * calculateMoment(points, numPoints, 1, 1) ) / ( calculateMoment(points, numPoints, 2, 0) - calculateMoment(points, numPoints, 0, 2) ) );
}

float LidarObstacleExtractor::measureLinearity(Point *points, int numPoints)
{
    int k = numPoints / 2;
//    Point cm = calculateCenterOfMass(points, numPoints);
    float angle = calculateAngleOfOrientation(points, numPoints);
    float M = tan(angle);
//    Point normal(-M,1);
    Point localNormals[k];
    for(int i = 0; i < k; i++)
    {
        Point a = points[rand() % numPoints];
        Point b;
        while(b == a)
        {
            b = points[rand() % numPoints];
        }
        float m = (b.y - a.y) / (b.x - a.x);
        float norm = sqrt(m*m + 1);
        float dp = m*M + 1;
        if(dp < 0)
        {
            localNormals[i] = Point(m/norm, -1/norm);
        } else {
            localNormals[i] = Point(-m/norm, 1/norm);
        }
    }
    Point normalToOrientation(0,0);
    for(int i = 0; i < k; i++)
    {
        normalToOrientation.x += localNormals[i].x;
        normalToOrientation.y += localNormals[i].y;
    }

    normalToOrientation.x /= numPoints;
    normalToOrientation.y /= numPoints;

    return normalToOrientation.distanceTo(Point(0,0));
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

}
}
}

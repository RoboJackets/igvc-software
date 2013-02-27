#include "lidarobstacleextractor.h"
#include <algorithm>
//#include <cmath>
#include <iostream>

using namespace IGVC::Sensors;

LidarObstacleExtractor::LidarObstacleExtractor()
{
}

std::vector<Obstacle*> LidarObstacleExtractor::extractObstacles(LidarState data)
{
    return extractLinearObstacles(&data);
}

/**
 * Runs hough linear transform on the given lidar data
 */
std::vector<Obstacle*> LidarObstacleExtractor::extractLinearObstacles(LidarState *data)
{
    using namespace std;

    vector<Obstacle *> lines;

    bool inLine = false;

    Point start, end;

    for(int i = 0; i < 1024; i++)
    {
        LidarPoint &pt = data->points[i];

        float x = cos(pt.angle)*pt.distance;
        float y = sin(pt.angle)*pt.distance;

        if(pt.valid)
        {
            if(!inLine)
            {
                start = Point(x, y);
                end = Point(x, y);
                inLine = true;
            } else {
                float dist = sqrt( (x-end.x)*(x-end.x) + (y-end.y)*(y-end.y) );
                if(dist > 0.05)
                {
                    if(start != end)
                    {
//                    end = Point(x,y);
                        lines.push_back(new LinearObstacle(start, end));
                    }
                    inLine = false;
                } else {
                    end = Point(x, y);
                }
            }
        } else {
            if(inLine)
            {
                lines.push_back(new LinearObstacle(start, end));
                inLine = false;
            }
        }
    }
    if(inLine)
    {
        lines.push_back(new LinearObstacle(start, end));
    }

    return lines;

/*  Hough transform
    float maxPossibleRho = 0;
    for(int i = 0; i < 1024; i++)
    {
        float rho = data->points[i].distance;
        maxPossibleRho = max(rho, maxPossibleRho);
    }

    float metersPerBin = 0.008;
    int numThetaBins = 100;
    int numRhoBins = maxPossibleRho / metersPerBin;
    int accumulator[numThetaBins][numRhoBins];
    for(int t = 0; t < numThetaBins; t++)
        for(int r = 0; r < numRhoBins; r++)
            accumulator[t][r] = 0;

    float radPerBin = ( M_PI * 2.0 ) / numThetaBins;

    for(int i = 0; i < 1024; i++)
    {
        LidarPoint &p = data->points[i];

        if(p.valid)
        {
            float x = cos(p.angle) * p.distance;
            float y = sin(p.angle) * p.distance;

            for(int t=0; t < numThetaBins; t++)
            {
                float theta = radPerBin * t;
                float rho = x * cos(theta) + y * sin(theta);
                if(rho > 0)
                {
                    int rhoIndex = rho / metersPerBin;
                    accumulator[t][rhoIndex]++;
                }
            }
        }
    }

    vector<Obstacle*> lines;

    // This is a totally eyeballed guess based on the data
    // we got from a robot standing at an eyeballed distance
    // from a wall in the shop. We think it'll work for the
    // fence. Could be wrong.
    int minVotes = 10;
    for(int t = 0; t < numThetaBins; t++)
    {
        for(int r = 0; r < numRhoBins; r++)
        {
            if(accumulator[t][r] > minVotes)
            {
                Point min;
                Point max;

                float theta = t * radPerBin;
                float rho = r * metersPerBin;

                bool foundMin = false;

                for(int i = 0; i < 1024; i++)
                {
                    LidarPoint &pt = data->points[i];
                    if(pt.valid)
                    {
                        float x = cos(pt.angle) * pt.distance;
                        float y = sin(pt.angle) * pt.distance;
                        bool isOnLine = isPointOnLine(x, y, rho, theta, i, data->points);
                        if(!foundMin && isOnLine)
                        {
                            min.x = x;
                            min.y = y;
                            foundMin = true;
                        } else if(foundMin && isOnLine)
                        {
                            max.x = x;
                            max.y = y;
                        } else if(foundMin && !isOnLine)
                        {
                            break;
                        }
                    } else if(foundMin)
                    {
                        break;
                    }
                }
                lines.push_back(new LinearObstacle(min, max));
            }
        }
    }

    return lines;*/
}

bool LidarObstacleExtractor::isPointOnLine(float x, float y, float rho, float theta, int pointIndex, LidarPoint *points)
{
    float y_dist_thresh = 0.1;
    float x_dist_thresh = 0.1;
    float n_theta_thresh = 0.01;
    if(sin(theta) != 0)
    {
        float y_line = (rho - cos(theta) * x) / sin(theta);
        if(abs(y_line - y) < y_dist_thresh)
        {
            return true;
        } else {
            return false;
        }

    } else {
        float n_theta = 0;
        if(abs(x-rho) < x_dist_thresh)
        {
            estimateNormalAtPoint(pointIndex, points, n_theta);
            if(abs(theta - n_theta) < n_theta_thresh)
            {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}

void LidarObstacleExtractor::estimateNormalAtPoint(int pointIndex, LidarPoint *points, float &n_theta)
{
    Point p1(points[pointIndex-1].distance*cos(points[pointIndex-1].angle),points[pointIndex-1].distance*sin(points[pointIndex-1].angle));
    Point p2(points[pointIndex+1].distance*cos(points[pointIndex+1].angle),points[pointIndex+1].distance*sin(points[pointIndex+1].angle));
    Point diff(p1.x-p2.x, p1.y-p2.y);
    n_theta = atan2(diff.y, diff.x);
}

void LidarObstacleExtractor::extractCircularObstacles(LidarState *data)
{

}

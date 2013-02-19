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
    int maxVotes = 0;
    float distThreshold = 0.1;
    for(int t = 0; t < numThetaBins; t++)
    {
        for(int r = 0; r < numRhoBins; r++)
        {
            maxVotes = max(maxVotes , accumulator[t][r]);
            if(accumulator[t][r] > minVotes)
            {
                Point min;
                Point max;

                float theta = t * radPerBin;
                float rho = r * metersPerBin;

                for(int i = 0; i < 1024; i++)
                {
                    LidarPoint &pt = data->points[i];
                    if(pt.valid)
                    {
                        float x = cos(pt.angle) * pt.distance;
                        float y = sin(pt.angle) * pt.distance;
                        float y_line = (sin(theta)!=0) ? (rho - cos(theta) * x) / sin(theta) : y;
                        if(abs(y - y_line) < distThreshold)
                        {
                            min.x = x;
                            min.y = y;
                            break;
                        }
                    }
                }

                for(int i = 1024; i > -1; i--)
                {
                    LidarPoint &pt = data->points[i];
                    if(pt.valid)
                    {
                        float x = cos(pt.angle) * pt.distance;
                        float y = sin(pt.angle) * pt.distance;
                        float y_line = (sin(theta)!=0) ? (rho - cos(theta) * x) / sin(theta) : y;
                        if(abs(y - y_line) < distThreshold)
                        {
                            max.x = x;
                            max.y = y;
                            break;
                        }
                    }
                }

//                min.x = -1.0;
//                max.x = 1.0;

//                if(sin(theta) != 0)
//                {
//                    min.y = (rho - cos(theta) * min.x) / sin(theta);
//                    max.y = (rho - cos(theta) * max.x) / sin(theta);
//                } else {
//                    min.y = -1.0;
//                    max.y = 1.0;
//                }

                lines.push_back(new LinearObstacle(min, max));
            }
        }
    }
    std::cout << maxVotes << std::endl;

    return lines;
}

void LidarObstacleExtractor::extractCircularObstacles(LidarState *data)
{

}

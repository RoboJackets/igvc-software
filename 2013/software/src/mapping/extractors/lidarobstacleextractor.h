#ifndef LIDAROBSTACLEEXTRACTOR_H
#define LIDAROBSTACLEEXTRACTOR_H

#include "sensors/lidar/Lidar.h"
#include "mapping/obstacles/linearobstacle.h"
#include <cmath>
#include <vector>

using namespace IGVC::Sensors;
using namespace IGVC::mapping::obstacles;

struct Line {
    float rho;
    float theta;
    float y(float x) {
        if(sin(theta) != 0)
        {
            return ( rho - x*cos(theta) ) / sin(theta);
        } else {
            return 0;
        }
    }
};

class LidarObstacleExtractor
{
public:
    LidarObstacleExtractor();

    std::vector<Obstacle*> extractObstacles(LidarState data);

private:

    std::vector<Obstacle*> extractLinearObstacles(LidarState *data);
    void extractCircularObstacles(LidarState *data);

};

#endif // LIDAROBSTACLEEXTRACTOR_H

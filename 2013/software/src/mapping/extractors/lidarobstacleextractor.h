#ifndef LIDAROBSTACLEEXTRACTOR_H
#define LIDAROBSTACLEEXTRACTOR_H

#include "events/Event.hpp"
#include "sensors/lidar/Lidar.h"
#include "mapping/obstacles/linearobstacle.h"
#include <cmath>
#include <vector>

using namespace IGVC::Sensors;
using namespace IGVC::mapping::obstacles;

class LidarObstacleExtractor
{
public:
    LidarObstacleExtractor(Lidar* device);

    std::vector<Obstacle*> extractObstacles(LidarState data);

    Event<std::vector<Obstacle*> > onNewData;

private:

    void onNewLidarData(LidarState data);
    LISTENER(LidarObstacleExtractor, onNewLidarData, LidarState);

    std::vector<Obstacle*> extractLinearObstacles(LidarState *data);
    void extractCircularObstacles(LidarState *data);
    bool isPointOnLine(float x, float y, float rho, float theta, int pointIndex, LidarPoint *points);
    void estimateNormalAtPoint(int pointIndex, LidarPoint *points, float &n_theta);

};

#endif // LIDAROBSTACLEEXTRACTOR_H

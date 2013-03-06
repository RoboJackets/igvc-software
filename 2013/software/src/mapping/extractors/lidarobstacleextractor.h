#ifndef LIDAROBSTACLEEXTRACTOR_H
#define LIDAROBSTACLEEXTRACTOR_H

#include "events/Event.hpp"
#include "sensors/lidar/Lidar.h"
#include "mapping/obstacles/linearobstacle.h"
#include <cmath>
#include <vector>
#include "mapping/obstacles/pointarrayobstacle.h"

using namespace IGVC::Sensors;
using namespace IGVC::mapping::obstacles;

namespace IGVC
{
namespace mapping
{
namespace extractors
{

class LidarObstacleExtractor
{
public:
    LidarObstacleExtractor(Lidar* device);

    Event<std::vector<Obstacle*> > onNewData;

    float jumpDistThreshold;
    float linearityThreshold;

private:

    void onNewLidarData(LidarState data);
    LISTENER(LidarObstacleExtractor, onNewLidarData, LidarState)

    float measureLinearity(Point* points, int numPoints);
    float calculateMoment(Point* points, int numPoints, int p, int q);
    Point calculateCenterOfMass(Point* points, int numPoints);
    float calculateAngleOfOrientation(Point* points, int numPoints);

    std::vector<PointArrayObstacle*> clusterByValidity(LidarState *data);
    std::vector<PointArrayObstacle*> clusterByJumps(std::vector<PointArrayObstacle*> data);
    std::vector<Obstacle*> filterLinearObstacles(std::vector<PointArrayObstacle*> data);

};

}
}
}


#endif // LIDAROBSTACLEEXTRACTOR_H

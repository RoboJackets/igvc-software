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

    void disconnect();

    Event<std::vector<Obstacle*> > onNewData;

    float jumpDistThreshold;
    float linearityThreshold;
    int sizeThreshold;

private:

    Lidar *_device;

    void onNewLidarData(LidarState data);
    LISTENER(LidarObstacleExtractor, onNewLidarData, LidarState)

    float measureLinearity(Vector2f* points, int numPoints);
    float calculateMoment(Vector2f* points, int numPoints, int p, int q);
    Vector2f calculateCenterOfMass(Vector2f* points, int numPoints);
    float calculateAngleOfOrientation(Vector2f* points, int numPoints);

    std::vector<PointArrayObstacle*> clusterByValidity(LidarState *data);
    std::vector<PointArrayObstacle*> clusterByJumps(std::vector<PointArrayObstacle*> data);
    std::vector<PointArrayObstacle*> filterBySize(std::vector<PointArrayObstacle*> data);
    std::vector<Obstacle*> filterLinearObstacles(std::vector<PointArrayObstacle*> data);
    std::vector<Obstacle*> filterCircularObstacles(std::vector<Obstacle*> data);

};

}
}
}


#endif // LIDAROBSTACLEEXTRACTOR_H

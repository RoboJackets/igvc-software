#ifndef LIDAROBSTACLEEXTRACTOR_H
#define LIDAROBSTACLEEXTRACTOR_H

#include "events/Event.hpp"
#include "sensors/lidar/Lidar.h"
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
    float maxAllowedRadius;

private:

    Lidar *_device;

    void onNewLidarData(LidarState data);
    LISTENER(LidarObstacleExtractor, onNewLidarData, LidarState)

    float calculateLinearCorrelationCoefficient(Vector2f* points, int numPoints);
    Vector2f closestPointToLineFromPoint(Vector2f &lineCoefs, Vector2f &p);

    std::vector<PointArrayObstacle*> clusterByValidity(LidarState *data);
    std::vector<PointArrayObstacle*> clusterByJumps(std::vector<PointArrayObstacle*> data);
    std::vector<PointArrayObstacle*> filterBySize(std::vector<PointArrayObstacle*> data);
    std::vector<Obstacle*> filterByLinearRegression(std::vector<PointArrayObstacle*> data);
    std::vector<Obstacle*> filterCircularObstacles(std::vector<Obstacle*> data);
    std::vector<Obstacle*> filterCirclesByRadius(std::vector<Obstacle*> data);

    std::vector<Obstacle*> filterCirclularObstacleWithRadius(std::vector<Obstacle*> data, float radius);

    std::vector<Obstacle*> smoothData(std::vector<PointArrayObstacle*> data);
};

}
}
}


#endif // LIDAROBSTACLEEXTRACTOR_H

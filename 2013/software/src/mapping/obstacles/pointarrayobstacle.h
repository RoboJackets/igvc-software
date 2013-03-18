#ifndef POINTARRAYOBSTACLE_H
#define POINTARRAYOBSTACLE_H

#include "mapping/obstacles/obstacle.hpp"
#include <vector>

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

class PointArrayObstacle : public Obstacle
{
public:
    PointArrayObstacle();
    int getNumPoints();
    Vector2f* getPoints();

    void addPoint(Vector2f p);

private:
    std::vector<Vector2f> _points;
};

}
}
}

#endif // POINTARRAYOBSTACLE_H

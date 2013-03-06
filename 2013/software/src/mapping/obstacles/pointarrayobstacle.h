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
    Point* getPoints();

    void addPoint(Point p);

private:
    std::vector<Point> _points;
};

}
}
}

#endif // POINTARRAYOBSTACLE_H

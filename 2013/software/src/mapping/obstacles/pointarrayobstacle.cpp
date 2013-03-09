#include "pointarrayobstacle.h"

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

PointArrayObstacle::PointArrayObstacle()
    : _points()
{
}

int PointArrayObstacle::getNumPoints()
{
    return _points.size();
}

Point* PointArrayObstacle::getPoints()
{
    return _points.data();
}

void PointArrayObstacle::addPoint(Point p)
{
    _points.push_back(p);
}

}
}
}

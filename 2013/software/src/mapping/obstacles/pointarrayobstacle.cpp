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

Vector2f *PointArrayObstacle::getPoints()
{
    return _points.data();
}

void PointArrayObstacle::addPoint(Vector2f p)
{
    _points.push_back(p);
}

}
}
}

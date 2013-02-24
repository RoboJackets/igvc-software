#include "linearobstacle.h"
#include <cmath>

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

LinearObstacle::LinearObstacle()
    : Obstacle()
{
}

LinearObstacle::LinearObstacle(Point start, Point end)
    : Obstacle()
{
    _start = start;
    _end = end;
}

Point* LinearObstacle::getPoints()
{
    Point *ps = new Point[2];
    ps[0] = _start;
    ps[1] = _end;
    return ps;
}

int LinearObstacle::getNumPoints()
{
    return 2;
}

void LinearObstacle::setPoints(Point start, Point end)
{
    _start = start;
    _end = end;
}

void LinearObstacle::setStart(Point start)
{
    _start = start;
}

void LinearObstacle::setEnd(Point end)
{
    _end = end;
}

}
}
}

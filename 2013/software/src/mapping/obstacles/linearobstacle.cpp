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

LinearObstacle::LinearObstacle(Vector2f start, Vector2f end)
    : Obstacle()
{
    _start = start;
    _end = end;
}

Vector2f *LinearObstacle::getPoints()
{
    Vector2f *ps = new Vector2f[2];
    ps[0] = _start;
    ps[1] = _end;
    return ps;
}

int LinearObstacle::getNumPoints()
{
    return 2;
}

void LinearObstacle::setPoints(Vector2f start, Vector2f end)
{
    _start = start;
    _end = end;
}

void LinearObstacle::setStart(Vector2f start)
{
    _start = start;
}

void LinearObstacle::setEnd(Vector2f end)
{
    _end = end;
}

}
}
}

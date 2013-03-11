#include "circularobstacle.h"

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

CircularObstacle::CircularObstacle()
{
    _center = Point(0,0);
    _radius = 0;
    _resolution = 8;
}

CircularObstacle::CircularObstacle(Point center, float radius)
{
    _center = center;
    _radius = radius;
    _resolution = 8;
}

CircularObstacle::CircularObstacle(Point center, float radius, int resolution)
{
    _center = center;
    _radius = radius;
    _resolution = resolution;
}

Point* CircularObstacle::getPoints()
{
    Point *pts = new Point[_resolution];

    int index = 0;
    for(float theta = 0; theta < 2*M_PI; theta += ( 2 * M_PI ) / _resolution)
    {
        pts[index] = Point(_radius*cos(theta)+_center.x, _radius*sin(theta)+_center.y);
        index++;
    }

    return pts;
}

int CircularObstacle::getNumPoints()
{
    return _resolution;
}

void CircularObstacle::resolution(int val)
{
    _resolution = val;
}

int CircularObstacle::resolution()
{
    return _resolution;
}

}
}
}

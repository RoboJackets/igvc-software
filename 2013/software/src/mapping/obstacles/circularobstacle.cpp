#include "circularobstacle.h"

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

CircularObstacle::CircularObstacle()
{
    _center = Vector2f(0.0f,0.0f);
    _radius = 0;
    _resolution = 8;
}

CircularObstacle::CircularObstacle(Vector2f center, float radius)
{
    _center = center;
    _radius = radius;
    _resolution = 8;
}

CircularObstacle::CircularObstacle(Vector2f center, float radius, int resolution)
{
    _center = center;
    _radius = radius;
    _resolution = resolution;
}

Vector2f* CircularObstacle::getPoints()
{
    Vector2f *pts = new Vector2f[_resolution];

    int index = 0;
    for(float theta = 0; theta <= 2*M_PI; theta += ( 2 * M_PI ) / _resolution)
    {
        pts[index] = Vector2f(_radius*cos(theta), _radius*sin(theta)) + _center;
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

void CircularObstacle::center(Vector2f val)
{
    _center = val;
}

Vector2f CircularObstacle::center()
{
    return _center;
}

void CircularObstacle::radius(float val)
{
    _radius = val;
}

float CircularObstacle::radius()
{
    return _radius;
}

}
}
}

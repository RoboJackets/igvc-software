#ifndef OBSTACLE_H
#define OBSTACLE_H

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

struct Point {
    Point() {
        x = 0;
        y = 0;
    }

    Point(float xval, float yval)
    {
        x = xval;
        y = yval;
    }

    bool operator == (Point other)
    {
        return x == other.x && y == other.y;
    }

    bool operator != (Point other)
    {
        return x != other.x || y != other.y;
    }

    float x;
    float y;
};

class Obstacle
{
public:
    Obstacle() { }

    virtual Point* getPoints() = 0;
    virtual int getNumPoints() = 0;

};

}
}
}



#endif // OBSTACLE_H

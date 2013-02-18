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

    Point(int xval, int yval)
    {
        x = xval;
        y = yval;
    }

    float x, y;
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

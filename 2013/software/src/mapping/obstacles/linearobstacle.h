#ifndef LINEAROBSTACLE_H
#define LINEAROBSTACLE_H

#include "mapping/obstacles/obstacle.hpp"

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

class LinearObstacle : public Obstacle
{
public:
    LinearObstacle();

    LinearObstacle(Point start, Point end);

    Point* getPoints();
    int getNumPoints();

    void setPoints(Point start, Point end);

    void setStart(Point start);
    void setEnd(Point end);

private:
    Point _start;
    Point _end;
};

}
}
}

#endif // LINEAROBSTACLE_H

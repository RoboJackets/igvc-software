#ifndef CIRCULAROBSTACLE_H
#define CIRCULAROBSTACLE_H

#include "mapping/obstacles/obstacle.hpp"

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

class CircularObstacle : public Obstacle
{
public:
    CircularObstacle();
    CircularObstacle(Point center, float radius);
    CircularObstacle(Point center, float radius, int resolution);
    Point* getPoints();
    int getNumPoints();

    void resolution(int val);
    int resolution();

    void center(Point val);
    Point center();

    void radius(float val);
    float radius();

private:
    int _resolution;
    Point _center;
    float _radius;
};

}
}
}

#endif // CIRCULAROBSTACLE_H

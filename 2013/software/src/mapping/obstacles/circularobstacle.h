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
    CircularObstacle(Vector2f center, float radius);
    CircularObstacle(Vector2f center, float radius, int resolution);
    Vector2f* getPoints();
    int getNumPoints();

    void resolution(int val);
    int resolution();

    void center(Vector2f val);
    Vector2f center();

    void radius(float val);
    float radius();

private:
    int _resolution;
    Vector2f _center;
    float _radius;
};

}
}
}

#endif // CIRCULAROBSTACLE_H

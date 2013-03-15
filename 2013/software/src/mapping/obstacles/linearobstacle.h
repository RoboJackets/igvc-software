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

    LinearObstacle(Vector2f start, Vector2f end);

    Vector2f* getPoints();
    int getNumPoints();

    void setPoints(Vector2f start, Vector2f end);

    void setStart(Vector2f start);
    void setEnd(Vector2f end);

private:
    Vector2f _start;
    Vector2f _end;
};

}
}
}

#endif // LINEAROBSTACLE_H

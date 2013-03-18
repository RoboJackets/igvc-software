#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace IGVC
{
namespace mapping
{
namespace obstacles
{

using namespace Eigen;

class Obstacle
{
public:
    Obstacle() { }

    virtual Vector2f* getPoints() = 0;
    virtual int getNumPoints() = 0;

};

}
}
}



#endif // OBSTACLE_H

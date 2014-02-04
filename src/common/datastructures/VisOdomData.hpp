#ifndef VISODOMDATA_H
#define VISODOMDATA_H

#include <math.h>

using namespace std;

class VisOdomData : public SensorData
{
public:
    inline VisOdomData()
    {
    }

    inline VisOdomData(double deltaX, double deltaY, double deltaTime, double variance = 123123) : SensorData(), _deltaX(deltaX),
     _deltaY(deltaY), _deltaTime(deltaTime), _Variance(variance)
    {
    }

    inline double deltaX(void)
    {
        return _deltaX;
    }

    inline double deltaY(void)
    {
        return _deltaY;
    }

    inline double Speed(void)
    {
        return sqrt(pow(_deltaX,2) + pow(_deltaY,2))/_deltaTime;
    }

    inline double Heading(void)
    {
      return atan2(_deltaY, _deltaX);
    }

    inline double deltaTime(void)
    {
        return _deltaTime;
    }

    ostream& print(ostream& stream)//, VisOdomData dat)
    {

      return stream << "Change in X is " << _deltaX << endl << " Change in Y is" << _deltaY << endl;
    }

private:
    double _deltaX; //East being positive
    double _deltaY; //North being positive
    double _deltaTime;
    double _Variance;
};

#endif // VISODOMDATA_H

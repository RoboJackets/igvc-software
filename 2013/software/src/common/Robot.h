
#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>

class Robot
{
  public:
    Robot();
    static Robot Misti();
    static Robot Roxii();
    double Baseline();
    double HeightOfMast();
    double Mast2Center();
    double HorizontalCamOffset();
    double TireDiameter();
    double TireRadius();
    double TireCircumference();
    double Lidar2Center();
    double CameraAngle();
    void Baseline(double);
    void HeightOfMast(double);
    void Mast2Center(double);
    void HorizontalCamOffset(double);
    void TireDiameter(double);
    void CameraAngle(double);
    void Lidar2Center(double);
    virtual ~Robot();
  private:
    double _Baseline;
    double _HeightOfMast;
    double _Mast2Center;
    double _HorizontalCamOffset;
    double _TireDiameter;
    double _CameraAngle;
    double _Lidar2Center;
};

#endif // ROBOT_H

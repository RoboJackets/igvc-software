#include "Robot.h"

Robot::Robot() : _Baseline(0), _HeightOfMast(0), _Mast2Center(0), _HorizontalCamOffset(0), _TireDiameter(0), _CameraAngle(0)
{
  //ctor
}

/**
Returns a Robot structure with all of the appropriate values for the 2013 robot loaded
**/
Robot Robot::Misti()
{
  Robot misti;
  misti.Baseline(1);
  misti.HeightOfMast(2);
  misti.Mast2Center(3);
  misti.HorizontalCamOffset(3.5);
  misti.TireDiameter(4);
  misti.CameraAngle(5);
  return misti;
}


/**
Returns a Robot structure with all of the appropriate values for the 2011-2012 robot loaded
**/
Robot Robot::Roxii()
{
  Robot roxi;
  roxi.Baseline(0);
  roxi.HeightOfMast(0);
  roxi.Mast2Center(0);
  roxi.HorizontalCamOffset(323);
  roxi.TireDiameter(0);
  roxi.CameraAngle(0);
  return roxi;
}


/**
Gives the distance between the center of the wheels on the left and right side of the robot
Units: Meters
**/
double Robot::Baseline()
{
  return(_Baseline);
}

double Robot::HeightOfMast()
{
  return _HeightOfMast;
}

/**
Gives the distance from the base of the mast(more precisely the point directly below where the camera sit0s) forward to the center of the robot
Units:Meters
**/
double Robot::Mast2Center()
{
  return(_Mast2Center);
}

/**
The horizontal distance from the focal point to the center of the mast/robot
Units: Meters
**/
double Robot::HorizontalCamOffset()
{
  return _HorizontalCamOffset;
}

/**
The outer diameter of the tire of the robot
Units: Meters
**/
double Robot::TireDiameter()
{
  return(_TireDiameter);
}

/**
The outer radius of the tire of the robot, derived from diameter
Units: Meters
**/
double Robot::TireRadius()
{
  return(_TireDiameter/2);
}

/**
The outer circumference of the tire of the robot, derived from diameter
Units: Meters
**/
double Robot::TireCircumference()
{
  return(_TireDiameter*M_PI);
}

/**
The angle that center of the vertical field of view makes with horizontal when the robot is sitting on flat ground
Units: Meters
**/
double Robot::CameraAngle()
{
  return _CameraAngle;
}

/**
The  distance from the LIDAR to the center of the robot(assumes LIDAR is centered)
**/
double Robot::Lidar2Center()
{
  return _Lidar2Center;
}


void Robot::Baseline(double newBaseline)
{
  _Baseline = newBaseline;
}

void Robot::HeightOfMast(double newHeightOfMast)
{
  _HeightOfMast = newHeightOfMast;
}

void Robot::Mast2Center(double newMast2Center)
{
  _Mast2Center = newMast2Center;
}

void Robot::HorizontalCamOffset(double newOffset)
{
  _HorizontalCamOffset = newOffset;
}

void Robot::TireDiameter(double newTireDiameter)
{
  _TireDiameter = newTireDiameter;
}

void Robot::CameraAngle(double newCameraAngle)
{
  _CameraAngle = newCameraAngle;
}

void Robot::Lidar2Center(double newLidar2Center)
{
  _Lidar2Center = newLidar2Center;
}
Robot::~Robot()
{
  //dtor
}


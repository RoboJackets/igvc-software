#include "Robot.h"

#define inchesToMeters(a) a*.0254
#define deg2rad(a) a/180*M_PI
#define rad2deg(a) a*180/M_PI

Robot::Robot() : _Baseline(0), _HeightOfMast(0), _Mast2Center(0), _HorizontalCamOffset(0), _TireDiameter(0), _CameraAngle(0)
{
  //ctor
}

Robot Robot::CurrentRobot()
{
  return Misti();
}

/**
Returns a Robot structure with all of the appropriate values for the 2013 robot loaded
**/
Robot Robot::Misti()
{
  Robot misti;
  misti.Baseline(inchesToMeters(24));
  misti.HeightOfMast(inchesToMeters(62));
  misti.Mast2Center(inchesToMeters(22));
  misti.HorizontalCamOffset(inchesToMeters(2.5));
  misti.TireDiameter(inchesToMeters(13.5));
  misti.Lidar2Center(inchesToMeters(12));
  misti.CameraAngle(deg2rad(22.5));
  misti.Length(inchesToMeters(40.375));
  return misti;
}


/**
Returns a Robot structure with all of the appropriate values for the 2011-2012 robot loaded
**/
Robot Robot::Roxii()
{
  Robot roxi;
  roxi.Baseline(inchesToMeters(29));
  roxi.HeightOfMast(inchesToMeters(63));
  roxi.Mast2Center(inchesToMeters(20));
  roxi.HorizontalCamOffset(inchesToMeters(2.5));
  roxi.TireDiameter(inchesToMeters(10.5));
  roxi.Lidar2Center(inchesToMeters(11));
  roxi.CameraAngle(deg2rad(22.5));
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

double Robot::Length()
{
  return _Length;
}

double Robot::Dist2Front()
{
  return _Length/2;
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

void Robot::Length(double len)
{
  _Length = len;
}

Robot::~Robot()
{
  //dtor
}


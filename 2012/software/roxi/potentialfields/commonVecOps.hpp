#ifndef _COMMONVECOPS_HPP_
#define _COMMONVECOPS_HPP

#include "math.h"

double rad2deg(double rads);
double deg2rad(double degs);
double vec2bear(double ang);
double bear2vec(double bear);
void xyToVec(double x, double y, double& mag, double& ang);
void VecToxy(double mag, double ang, double& x, double& y);
void AddVecs(double* xvals, double* yvals, int numVecs, double& xnet, double& ynet);
double Distance2D(double x1, double y1, double x2, double y2);
double RotateBearing(double angle0, double dangle);
double clampVector(double ang, double gps_clamp_angle);

#endif

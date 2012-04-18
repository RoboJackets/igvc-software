#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "RobotPosition.hpp"
CvMat* getRobotToWorldMat();
CvMat* getWorldToRobotMat();
CvMat* getRobotToWorldMat(RobotPosition robotAt);
CvMat* getWorldToRobotMat(RobotPosition robotAt);
void convertRobotToWorld(double *x, double *y, RobotPosition pos);
void convertWorldToRobot(double *x, double *y, RobotPosition pos);
void convertRobotToWorld(double *x, double *y);
void convertWorldToRobot(double *x, double *y);




#ifndef IMAGEUTILS_H
#define IMAGEUTILS_H

#include <list>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "common/Robot.h"
#include "sensors/camera3D/CameraInfo.h"

using namespace cv;
using namespace Eigen;

void computeOffsets(vector<KeyPoint>& keypoints, MatrixXd& Pos, Robot& derRobot, CameraInfo& derCameraInfo, int nRows, int nCols);
Matrix3d centerImageCoords(int nRows, int nCols);
Matrix3d RollRotMatrix(double roll);
Matrix3d PitchRotMatrix(double pitch);
Matrix3d YawRotMatrix(double yaw);
Matrix4d HomogRollRotMatrix(double roll);
Matrix4d HomogPitchRotMatrix(double pitch);
Matrix4d HomogYawRotMatrix(double yaw);
Matrix3d RotMat3d(double roll, double pitch, double yaw);
Matrix4d HomogRotMat3d(double roll, double pitch, double yaw);
Matrix2d ImgRotMat(double angle);
Matrix3d HomogImgRotMat(double angle);



#endif // IMAGEUTILS_H

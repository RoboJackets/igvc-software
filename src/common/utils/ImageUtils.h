#ifndef IMAGEUTILS_H
#define IMAGEUTILS_H

#include <list>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <hardware/sensors/camera/CameraInfo.h>
#include <common/utils/timing.h>

using namespace cv;
using namespace Eigen;

void computeOffsets(vector<KeyPoint>& keypoints, MatrixXd& Pos, IGVC::CameraInfo& derCameraInfo, int nRows, int nCols);
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
Mat correctDistortion(Mat rawImg, Mat camerMatrix, Mat _distCoeffs);



#endif // IMAGEUTILS_H

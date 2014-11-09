#ifndef IMAGEUTILS_H
#define IMAGEUTILS_H

#include <list>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <common/utils/timing.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void computeOffsets(cv::vector<cv::KeyPoint>& keypoints, Eigen::MatrixXd& Pos, int nRows, int nCols);
Eigen::Matrix3d centerImageCoords(int nRows, int nCols);
Eigen::Matrix3d RollRotMatrix(double roll);
Eigen::Matrix3d PitchRotMatrix(double pitch);
Eigen::Matrix3d YawRotMatrix(double yaw);
Eigen::Matrix4d HomogRollRotMatrix(double roll);
Eigen::Matrix4d HomogPitchRotMatrix(double pitch);
Eigen::Matrix4d HomogYawRotMatrix(double yaw);
Eigen::Matrix3d RotMat3d(double roll, double pitch, double yaw);
Eigen::Matrix4d HomogRotMat3d(double roll, double pitch, double yaw);
Eigen::Matrix2d ImgRotMat(double angle);
Eigen::Matrix3d HomogImgRotMat(double angle);
cv::Mat correctDistortion(cv::Mat rawImg, cv::Mat camerMatrix, cv::Mat _distCoeffs);
void transformPoints(cv::Mat &src, cv::Mat &dst);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(cv::Mat src);



#endif // IMAGEUTILS_H

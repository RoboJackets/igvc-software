#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <string>

#define upperLeftXImage 0
#define upperLeftYImage 0
#define upperRightXImage 640
#define upperRightYImage 0
#define lowerRightXImage 640
#define lowerRightYImage 360
#define lowerLeftXImage 0
#define lowerLeftYImage 360

#define upperLeftXWorld -84.5
#define upperLeftYWorld -66
#define upperLeftZWorld 120.5
#define upperRightXWorld 35
#define upperRightYWorld -54.75
#define upperRightZWorld 120.5
#define lowerRightXWorld 33.5
#define lowerRightYWorld -3.5
#define lowerRightZWorld 120.5
#define lowerLeftXWorld 67.5
#define lowerLeftYWorld 0
#define lowerLeftZWorld 100

int main()
{
  // Camera Matrix
  cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);
  cameraMatrix.at<double>(0, 0) = 833.532112;
  cameraMatrix.at<double>(0, 2) = 352.183485;
  cameraMatrix.at<double>(1, 1) = 837.763183;
  cameraMatrix.at<double>(1, 2) = 264.724999;
  cameraMatrix.at<double>(2, 2) = 1.000000;

  std::cout << "Initial camera matrix: " << cameraMatrix << std::endl;

  // Distortion coefficients
  cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0.072489;
  distCoeffs.at<double>(1) = 0.109291;
  distCoeffs.at<double>(2) = 0.014877;
  distCoeffs.at<double>(3) = 0.003510;
  distCoeffs.at<double>(4) = 0.000000;

  std::cout << "Distortion coefficients: " << distCoeffs << std::endl;

  // Output rotation and translation vectors
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat rMat(3, 3, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::Mat cameraToWorld(3, 4, cv::DataType<double>::type);

  // Image coordinates of object points
  std::vector<cv::Point2f> imagePoints;
  imagePoints.push_back(cv::Point2f(upperLeftXImage, upperLeftYImage));
  imagePoints.push_back(cv::Point2f(upperRightXImage, upperRightYImage));
  imagePoints.push_back(cv::Point2f(lowerRightXImage, lowerRightYImage));
  imagePoints.push_back(cv::Point2f(lowerLeftXImage, lowerLeftYImage));

  // World coordinates of object points
  std::vector<cv::Point3f> objectPoints;
  objectPoints.push_back(cv::Point3f(upperLeftXWorld, upperLeftYWorld, upperLeftZWorld));
  objectPoints.push_back(cv::Point3f(upperRightXWorld, upperRightYWorld, upperRightZWorld));
  objectPoints.push_back(cv::Point3f(lowerRightXWorld, lowerRightYWorld, lowerRightZWorld));
  objectPoints.push_back(cv::Point3f(lowerLeftXWorld, lowerLeftYWorld, lowerLeftZWorld));

  // solvepnp!
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

  // RODRIGUES THIS VECTOR
  cv::Rodrigues(rvec, rMat);

  // Get CameraToWlrd matrix
  cv::hconcat(rMat.inv(), -1 * rMat.inv() * tvec, cameraToWorld);

  std::cout << "rMat: " << rMat << std::endl;
  std::cout << "tvec: " << tvec << std::endl;
  std::cout << "final mat: " << cameraToWorld << std::endl;

  return 0;
}
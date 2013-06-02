#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "common/Robot.h"
#include "sensors/camera3D/CameraInfo.h"
#include "common/utils/ImageUtils.h"

#define inch2Meter(a) a*.054
double distxinInches = 113;
double distyinInches = 28;

double distx = 2.87;
double disty = 0.7112;


/*
int c = 650;
int r = 480;
*/

int c = 680;
int r = 500;

using namespace std;
using namespace cv;

/*
main()
{
  FileStorage fs("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/out_camera_data.xml", FileStorage::READ); // Read the settings
  Mat cameraMatrix, distCoeffs, corrected;
  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;
  Mat rawImg = imread("/home/alex/Desktop/IGVC/2013/software/trainingSets/_right3.jpg");
  corrected = correctDistortion(rawImg, cameraMatrix, distCoeffs);
  namedWindow("flatterblast", 1);
  imshow("flatterblast" , corrected);
  //waitKey(0);
  imwrite("/home/alex/Desktop/IGVC/2013/software/trainingSets/calib_right3.jpg", corrected);
}
*/



main()
{
  vector<KeyPoint> entry;

  entry.push_back(KeyPoint(c,r,1,1,1,1,1));
  cout << entry.at(0).pt.x << endl;
  Eigen::MatrixXd datAnswer;
  Robot rob = Robot::CurrentRobot();
  IGVC::CameraInfo inf = IGVC::CameraInfo::CurrentCamera();
  computeOffsets(entry, datAnswer, rob, inf, 768, 1024);
  cout << datAnswer;
}

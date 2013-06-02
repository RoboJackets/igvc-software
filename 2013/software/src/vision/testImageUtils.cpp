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


int c = 680;
int r = 500;


using namespace std;
using namespace cv;

main()
{
  FileStorage fmx1("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/mx1.xml", FileStorage::READ);
  FileStorage fmx2("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/mx2.xml", FileStorage::READ);
  FileStorage fmy1("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/my1.xml", FileStorage::READ);
  FileStorage fmy2("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/my2.xml", FileStorage::READ);
  Mat left = imread("/home/alex/Desktop/img_left2.jpg");
  mat newLeft
  Settings
  remap(left, leftCalib, mx1, my1);
  if (!fmx1.isOpened())
  {
    cout << "could not open config file" << endl;
  }
}

/*
main()
{
  vector<KeyPoint> entry;

  entry.push_back(KeyPoint(c,r,1,1,1,1,1));
  cout << entry.at(0).pt.x << endl;
  Eigen::MatrixXd datAnswer;
  Robot rob = Robot::CurrentRobot();
  CameraInfo inf = CameraInfo::CurrentCamera();
  computeOffsets(entry, datAnswer, rob, inf, 768, 1024);
  cout << datAnswer;

}
*/

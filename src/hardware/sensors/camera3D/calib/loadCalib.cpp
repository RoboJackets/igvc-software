
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/utils/ImageUtils.h"


using namespace cv;
using namespace std;

main()
{
  FileStorage fs("/home/alex/Desktop/IGVC/2013/software/src/sensors/camera3D/calib/out_camera_data.xml", FileStorage::READ); // Read the settings
  //Settings s;

  if (!fs.isOpened())
  {
      cout << "Could not open the configuration file" << endl;
      return -1;
  }

  Mat cameraMatrix, distCoeffs;

  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;
  Mat view, rview, map1, map2;
  view = imread("/home/alex/Desktop/img_left2.jpg", 1);
  if(view.empty())
  {
    cout << "view had no data" << endl;
  }

  /*
  Size imageSize = view.size();

  Mat optNewMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);
  cout << "optNewMat passed" << endl;
  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), optNewMat, imageSize, CV_16SC2, map1, map2);


  remap(view, rview, map1, map2, INTER_LINEAR);
  */

  rview = correctDistortion(view, cameraMatrix, distCoeffs);


  imshow("Image View", rview);
  char c = (char)waitKey();
  if(c == 'q' || c == 'Q' )
    return 7;
}

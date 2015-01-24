#include "ImageUtils.h"
#include <common/config/configmanager.h>

using namespace std;
using namespace cv;
using namespace Eigen;

void computeOffsets(vector<KeyPoint>& keypoints, MatrixXd& Pos, int nRows, int nCols)
{

  //Compute Position Information for points

  double cameraHeight;
  double yCam, xCam, yRobot, xRobot, zRobot;
  double roll, pitch, yaw;
  roll = pitch = yaw = 0;
  Vector3d pos;
  int r,c;
  Eigen::Matrix<double,4,4> rotDynMat;
  Eigen::MatrixXd currentPos(4,keypoints.size());
  Vector3d cameraPos, cameraOffset;

  double phi, theta;
  cameraPos << - ConfigManager::Instance().getValue("Dimensions", "Mast2Center", 0), 0, - ConfigManager::Instance().getValue("Dimensions", "HeightOfMast", 0); //both variables are negated because of NED direction conventions

  rotDynMat = HomogRotMat3d(roll, pitch, yaw);

  cameraOffset = rotDynMat.topLeftCorner(3,3)*cameraPos; //describes position relative to
  cameraHeight = -cameraOffset(2);

  //Need to get the positions for all points, not just those matched, because non-matched may appear in next frame
  for(unsigned int i=0;i<keypoints.size();i++)
  {
    r = keypoints[i].pt.y;
    c = keypoints[i].pt.x;
    pos << r,c,1; //BE CAREFUL, REMEMBER R CORRESPONDS TO THE Y VALUE

    pos = centerImageCoords(nRows, nCols) * pos; //Changes coordinates in picture such that center of image is 0,0
    pos = HomogImgRotMat(roll)*pos; //Correct for roll of image by rotating it back the opposite way


    phi = ConfigManager::Instance().getValue("Dimensions", "CameraAngle", 0) - pitch + atan2((ConfigManager::Instance().getValue("Camera", "PixelSideLength", 0) * pos(0)), ConfigManager::Instance().getValue("Camera", "FocalLength", 0));

    theta = atan2((ConfigManager::Instance().getValue("Camera", "PixelSideLength", 0) * pos(1)), ConfigManager::Instance().getValue("Camera", "FocalLength", 0));


    xCam = cameraHeight/tan(phi);

    yCam = xCam*tan(theta);

    std::cout << "xcam is " << xCam << ". yCam is " << yCam << std::endl;
    xRobot = xCam + cameraOffset(0);

    yRobot = yCam + cameraOffset(1);
    zRobot = 0;  //Since we are defining the robot position as on the ground and the points are as welll

    //Get positions of objects relative to camera

    currentPos(0,i) = xRobot;
    currentPos(1,i) = yRobot;
    currentPos(2,i) = zRobot;
    currentPos(3,i) = 1;
  }

  currentPos = rotDynMat.inverse()*currentPos; //makes positions extrinsic. i.e. x is now north, etc
  Pos = currentPos;

}


Matrix3d RollRotMatrix(double roll)
{
  Matrix3d rMat = MatrixXd::Zero(3,3);
  rMat(0,0) = 1;
  rMat(1,1) = cos(roll);
  rMat(1,2) = sin(roll);
  rMat(2,1) = -sin(roll);
  rMat(2,2) = cos(roll);
  return rMat;
}

Matrix4d HomogRollRotMatrix(double roll)
{
  Matrix4d rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = RollRotMatrix(roll);
  rMat(3,3) = 1;
  return rMat;
}

Matrix3d PitchRotMatrix(double pitch)
{
  Matrix3d rMat = MatrixXd::Zero(3,3);
  rMat(0,0) = cos(pitch);
  rMat(0,2) = sin(pitch);
  rMat(1,1) = 1;
  rMat(2,0) = -sin(pitch);
  rMat(2,2) = cos(pitch);
  return rMat;
}

Matrix4d HomogPitchRotMatrix(double pitch)
{
  Matrix4d rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = PitchRotMatrix(pitch);
  rMat(3,3) = 1;
  return rMat;
}

Matrix3d YawRotMatrix(double yaw)
{
  MatrixXd rMat = MatrixXd::Zero(3,3);
  rMat(0,0) = cos(yaw);
  rMat(0,1) = sin(yaw);
  rMat(1,0) = -sin(yaw);
  rMat(1,1) = cos(yaw);
  rMat(2,2) = 1;
  return rMat;
}

Matrix4d HomogYawRotMatrix(double yaw)
{
  Matrix4d rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = YawRotMatrix(yaw);
  rMat(3,3) = 1;
  return rMat;
}

Matrix3d RotMat3d(double roll, double pitch, double yaw)
{
  return YawRotMatrix(yaw)*PitchRotMatrix(pitch)*RollRotMatrix(roll);
}

Matrix4d HomogRotMat3d(double roll, double pitch, double yaw)
{
  return HomogYawRotMatrix(yaw)*HomogPitchRotMatrix(pitch)*HomogRollRotMatrix(roll);
}


Matrix3d centerImageCoords(int nRows, int nCols)
{
  Matrix3d transMat = MatrixXd::Identity(3,3);
  transMat(0,0) = 1;
  transMat(1,1) = 1;
  transMat(0,2) = -(nRows-1)/2;
  transMat(1,2) = -(nCols-1)/2;

  return transMat;
}

//Assumes vector it is multiplying has row value in first row, column value in second row
Matrix2d ImgRotMat(double angle)
{
  Matrix2d rotMat;
  rotMat(0,0) = cos(angle);
  rotMat(0,1) = sin(angle);
  rotMat(1,0) = -sin(angle);
  rotMat(1,1) = cos(angle);
  return rotMat;
}

Matrix3d HomogImgRotMat(double angle)
{
  Matrix3d rotMat = MatrixXd::Zero(3,3);
  rotMat.topLeftCorner(2,2) = ImgRotMat(angle);
  rotMat(2,2) = 1;
  return rotMat;
}

Mat correctDistortion(Mat rawImg, Mat cameraMatrix, Mat distCoeffs)
{
  /*
  double start, end;
  start = seconds_since_IGVCpoch();
  */

  Mat rview, map1, map2;

  Size imageSize = rawImg.size();

  Mat optNewMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);

  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), optNewMat, imageSize, CV_16SC2, map1, map2);

  remap(rawImg, rview, map1, map2, INTER_LINEAR);
  /*
  end = seconds_since_IGVCpoch();
  std::cout << "This function took " << end-start << " seconds." << std::endl;
  */
  return rview;
}

void transformPoints(Mat &src, Mat &dst){
    //pcam is where the coordinates are in actual space (in meters right now)
    //pcam = (cv::Mat_<float>(4,2) << offset-12,72, offset, 72, offset, 60,offset -12, 60);
   // pcam = (cv::Mat_<float>(4,2) << 4,81, -8, 81, -8, 93,4, 93);
    int squareSize = ConfigManager::Instance().getValue("perspectiveTransform", "SquareSize", 100);
    Mat pcam = (cv::Mat_<float>(4,2) << dst.cols/2 - (squareSize/2),dst.rows-squareSize, dst.cols/2+(squareSize/2), dst.rows-squareSize, dst.cols/2-(squareSize/2), dst.rows - squareSize*2, dst.cols/2+(squareSize/2), dst.rows - squareSize*2);
    //p is where they show up as pixels on the camera
    //p = (cv::Mat_<float>(4,2) << 427, 642, 515, 642, 512, 589, 432, 588);
    // p= (cv::Mat_<float>(4,2) << 440, 674, 356, 679, 364, 631, 439, 627);
    float ratioRows = (float) src.rows/768;
    float ratioCols = (float) src.cols/1024;
    Mat  p= (cv::Mat_<float>(4,2) << 344*ratioCols, 646*ratioRows, 668*ratioCols, 636*ratioRows, 415*ratioCols, 496*ratioRows, 619*ratioCols, 488*ratioRows);
    //pcam = pcam*3+450; //This is just so we can see it on the screen
    //Getting the transform
    Mat transformMat = cv::getPerspectiveTransform(p, pcam);
    //Apply the transform to dst and store result in dst
    cv::warpPerspective(src, dst, transformMat, dst.size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(Mat src){
    int squareSize = ConfigManager::Instance().getValue("perspectiveTransform", "SquareSize", 100);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //Add points to the cloud if they are white (right now only checking the first layer)

    for (int r=0; r<src.rows;r++){
        for (int c=0; c<src.cols; c++){
            if (src.at<cv::Vec3b>(r,c)[0]==255){
                float x = ( c - ( src.cols/2. ) ) / (float)squareSize;
                float y = ( src.rows - r ) / (float)squareSize;
                cloud->points.push_back(pcl::PointXYZ(x, y, 0));

            }
        }
    }
    return cloud;
}

#include "vision/GrassOdometer.h"

GrassOdometer::GrassOdometer(ColorRange limits, int numKeyPoints) : _colors(limits), _numKeyPoints(numKeyPoints)
{
}

void GrassOdometer::processImage(Mat src, int numPoints)
{
}

void GrassOdometer::



void GrassOdometer::ProcesImageSURF(Mat& frame)
{
  //Change image to grayscale for feature detection & description
  Mat frame_gray;
  cvtColor( frame, frame_gray, CV_BGR2GRAY );

  //Detect features
  int minHessian = 100;
  SurfFeatureDetector detector(minHessian);
  vector<KeyPoint> keypoints;
  detector.detect( frame_gray, keypoints);

  //Remove the points from non-grassy regions
  RemoveNonGrassPts(frame, keypoints);

  //Remove the points that are too far away from the robot
  ///TODO Add this functionality

  //Get feature descriptors
  SurfDescriptorExtractor extractor;
  Mat descriptors;
  extractor.compute(frame_gray, keypoints, descriptors);

  //Compute Position Information for points

  int nCols, nRows;
  nRows = 768;
  nCols = 1024;
  double dPhi,dTheta;
  double cameraAngle;
  double heightOfMast, cameraHeight;
  double yCam, xCam, zCam, yRobot, xRobot, zRobot;
  double roll, pitch, yaw;
  roll = pitch = yaw = 0;
  Vector3d pos;
  int r,c;
  Eigen::Matrix<double,4,4> rotDynMat;
  Eigen::MatrixXd currentPos(4,keypoints.size());
  Vector3d cameraPos, cameraOffset;
  double mastHeight, d2c;
  double phi, theta;
  d2c = 0.508; //meters
  mastHeight = 1.676; //meters
  cameraPos << -d2c, 0, -mastHeight; //both d2c and mastHeight are negative because of NED direction conventions

  cameraOffset = rotDynMat.topLeftCorner(3,3)*cameraPos; //describes position relative to
  cameraHeight = -cameraOffset(2);

  rotDynMat = HomogRotMat3d(roll, pitch, yaw);
  //Need to get the positions for all points, not just those matched, because non-matched may appear in next frame
  for(int i=0;i<keypoints.size();i++)
  {
    r = keypoints[i].pt.y;
    c = keypoints[i].pt.x;
    pos << r,c,1; //BE CAREFUL, REMEMBER R CORRESPONDS TO THE Y VALUE

    pos = centerImageCoords(nRows, nCols) * pos; //Changes coordinates in picture such that center of image is 0,0
    pos = HomogImgRotMat(roll)*pos; //Correct for roll of image by rotating it back the opposite way

    phi = cameraAngle - pitch + dPhi * pos(0);
    theta = dTheta * pos(1);

    xCam = cameraHeight/tan(phi);
    yCam = cameraHeight/tan(theta);

    xRobot = xCam - cameraOffset(0);
    yRobot = yCam - cameraOffset(1);
    zRobot = 0;  //Since we are defining the robot position as on the ground and the points are as welll

    //Get positions of objects relative to camera

    currentPos(0,i) = xRobot;
    currentPos(1,i) = yRobot;
    currentPos(2,i) = zRobot;
    currentPos(3,i) = 1;
  }

  currentPos = rotDynMat.inverse()*currentPos; //makes positions extrinsic. i.e. x is now north, etc

  std::vector< DMatch > matches;

  MatchPointsFLANN(matches, descriptors, _lastFrameDescriptors);

  //Create matrices of corresponding points
  int nMatches = matches.size();
  MatrixXd oldPoints, newPoints, deltaMat;
  oldPoints = MatrixXd::Zero(4, nMatches);
  newPoints = MatrixXd::Zero(4, nMatches);
  int newIndex, oldIndex;
  for(int i =0;i<nMatches;i++)
  {
    newIndex = matches[i].queryIdx;
    oldIndex = matches[i].trainIdx;
    newPoints.col(i) = currentPos.col(newIndex);
    oldPoints.col(i) = _lastFramePositions.col(oldIndex);
  }

  deltaMat = newPoints-oldPoints;
  double deltax = deltaMat.row(0).mean();
  double deltay = deltaMat.row(1).mean();

  _lastFramePositions = currentPos;
  _lastFrameDescriptors = descriptors;
  _lastFrame = frame.clone();

}

Matrix3d GrassOdometer::RollRotMatrix(double roll)
{
  Matrix3d rMat = MatrixXd::Zero(3,3);
  rMat(0,0) = 1;
  rMat(1,1) = cos(roll);
  rMat(1,2) = sin(roll);
  rMat(2,1) = -sin(roll);
  rMat(2,2) = cos(roll);
  return rMat;
}

Matrix4d GrassOdometer::HomogRollRotMatrix(double roll)
{
  Matrix4d rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = RollRotMatrix(roll);
  rMat(3,3) = 1;
  return rMat;
}

Matrix3d GrassOdometer::PitchRotMatrix(double pitch)
{
   Matrix3d rMat = MatrixXd::Zero(3,3);
   rMat(0,0) = cos(pitch);
   rMat(0,2) = sin(pitch);
   rMat(1,1) = 1;
   rMat(2,0) = -sin(pitch);
   rMat(2,2) = cos(pitch);
}

Matrix4d GrassOdometer::HomogPitchRotMatrix(double pitch)
{
  Matrix3d rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = PitchRotMatrix(pitch);
  rMat(3,3) = 1;
}

Matrix3d GrassOdometer::YawRotMatrix(double yaw)
{
  MatrixXd rMat = MatrixXd::Zero(3,3);
  rMat(0,0) = cos(yaw);
  rMat(0,1) = sin(yaw);
  rMat(1,0) = -sin(yaw);
  rMat(1,1) = cos(yaw);
  rMat(2,2) = 1;
}

Matrix4d GrassOdometer::HomogYawRotMatrix(double yaw)
{
  MatrixXd rMat = MatrixXd::Zero(4,4);
  rMat.topLeftCorner(3,3) = YawRotMatrix(yaw);
  rMat(3,3) = 1;
}

Matrix3d GrassOdometer::RotMat3d(double roll, double pitch, double yaw)
{
  return YawRotMatrix(yaw)*PitchRotMatrix(pitch)*RollRotMatrix(roll);
}

Matrix4d GrassOdometer::HomogRotMat3d(double roll, double pitch, double yaw)
{
  return HomogYawRotMatrix(yaw)*HomogPitchRotMatrix(pitch)*HomogRollRotMatrix(roll);
}


Matrix3d GrassOdometer::centerImageCoords(int nRows, int nCols)
{
  Matrix3d transMat = MatrixXd::Identity(3,3);
  transMat(0,0) = 1;
  transMat(1,1) = 1;
  transMat(0,2) = -(nRows-1)/2;
  transMat(1,2) = -(nCols-1)/2;

  return transMat;
}

//Assumes vector it is multiplying has row value in first row, column value in second row
Matrix2d GrassOdometer::ImgRotMat(double angle)
{
  Matrix2d rotMat;
  rotMat(0,0) = cos(angle);
  rotMat(0,1) = sin(angle);
  rotMat(1,0) = -sin(angle);
  rotMat(1,1) = cos(angle);
  return rotMat;
}

Matrix3d GrassOdometer::HomogImgRotMat(double angle)
{
  Matrix3d rotMat = MatrixXd::Zero(3,3);
  rotMat.topLeftCorner(2,2) = ImgRotMat(angle);
  rotMat(2,2) = 1;
  return rotMat;
}



void GrassOdometer::RemoveNonGrassPts(Mat& frame, std::vector<KeyPoint>& keypoints)
{
  //Remove out of range points
  int r,c, nRows, nCols;
  uchar* pixel1;
  uchar* pixel2;

  for(int i=0;i<keypoints.size();i++)
  {
    r = keypoints[i].pt.y;
    c = keypoints[i].pt.x;
    pixel1 = &frame.data[frame.step[0]*r + frame.step[1]* c + 0];
    if (_colors.inRangeBGR(pixel1) != true)
    {
      keypoints.erase(keypoints.begin()+i);
      i--;
    }
  }
}

void GrassOdometer::MatchPointsFLANN(std::vector<DMatch>& good_matches, Mat& descriptors_1, Mat& descriptors_2 )
{
  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance < 2*min_dist )
    { good_matches.push_back( matches[i]); }
  }

}



void GrassOdometer::ShowCorrespondence(Mat& frame1, std::vector<KeyPoint>& keypoints_1, Mat& frame2, std::vector<KeyPoint>& keypoints_2,
                                  std::vector<DMatch>&  good_matches)
{
  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( frame1, keypoints_1, frame2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );

  for( int i = 0; i < good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

  waitKey(0);
}




GrassOdometer::~GrassOdometer()
{
}

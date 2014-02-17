#include "GrassOdometer.h"

GrassOdometer::GrassOdometer(ColorRange limits, int numKeyPoints) : _colors(limits), _numKeyPoints(numKeyPoints), _firstFrame(true)
{
  _cam = CameraInfo::CurrentCamera();
}

/**
Usage: Takes in iamge and produces an update statement for RobotPositon

**/

void GrassOdometer::processImage(ImageData src)
{
  Mat descriptors;
  vector<KeyPoint> kp;
  MatrixXd pos;
  double dx, dy, dt;
  VisOdomData update;
  dt = src.time() - _lastFrame.time();
  dy = dx = 0;
  if (_firstFrame)
  {
    _firstFrame=false;
  }
  else
  {
    findKeypointsSURF(src.mat(), kp, descriptors, pos);
    findDeltas(descriptors, pos, dx, dy);
    update = VisOdomData(dx,dy,dt);
  }
  _lastFramePositions = pos;
  _lastFrameDescriptors = descriptors;
  _lastFrame = src.deepCopy();
  update.print(std::cout);
  onNewData(update);

}

/**
Uses SURF method to find and identify keypoints and match them to previous frame's
**/

void GrassOdometer::findKeypointsSURF(Mat& frame, vector<KeyPoint>& theKeyPoints, Mat& theDescriptors, MatrixXd& thePositions)
{
   //Change image to grayscale for feature detection & description
  Mat frame_gray;
  cvtColor( frame, frame_gray, CV_BGR2GRAY );

  //Detect features

  SurfFeatureDetector detector(_numKeyPoints);
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
  nRows = frame.rows;
  nCols = frame.cols;
  MatrixXd pos;
  computeOffsets(keypoints, pos, _cam, nRows, nCols);
  thePositions = pos;
}

/**
Returns an average chnange of the row nad column values of keypoints within the iamge

**/
void GrassOdometer::findDeltas(Mat& newDescriptors, MatrixXd& newPos, double& deltax, double deltay)
{

  std::vector< DMatch > matches;

  MatchPointsFLANN(matches, newDescriptors, _lastFrameDescriptors);

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
    newPoints.col(i) = newPos.col(newIndex);
    oldPoints.col(i) = _lastFramePositions.col(oldIndex);
  }


  deltaMat = newPoints-oldPoints;
  //TODO use a better method than simply taking the mean to determine the change in position, consider some ML stuff
  deltax = deltaMat.row(0).mean();
  deltay = deltaMat.row(1).mean();
}

/**
Runs keypoints through a naiive color segmentation comparison to determine if they should be considered or are on an obstacle
**/
void GrassOdometer::RemoveNonGrassPts(Mat& frame, std::vector<KeyPoint>& keypoints)
{
  //Remove out of range points
  int r,c;
  uchar* pixel1;

  for(unsigned int i=0;i<keypoints.size();i++)
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

  for(unsigned int i = 0; i < good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

  waitKey(0);
}

GrassOdometer::~GrassOdometer()
{
}

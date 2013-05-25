#ifndef GRASSODOMETER_H
#define GRASSODOMETER_H

#include <list>
#include <eigen3/Eigen/Dense>

#include "vision/ColorRange.h"
//#include "common/FLPriotityQueue.hpp"
#include "common/Robot.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"


using namespace cv;
using namespace Eigen;

class GrassOdometer
{
  public:
    GrassOdometer(ColorRange limits, int numKeyPoints);
    void processImage(Mat src, int numPoints);
    void findKeypointsSURF(Mat& frame, vector<KeyPoint>& theKeyPoints, Mat& theDescriptors, MatrixXd& thePositions);
    void findDeltas(Mat& newDescriptors, MatrixXd& newPos, double& deltax, double deltay);
    void FuckItWeWillDoItLive(Mat& frame1, Mat& frame2);
    void ProcesImageSURF(Mat& frame);
    void RemoveNonGrassPts(Mat& frame, std::vector<KeyPoint>& keypoints);
    void ShowCorrespondence(Mat& frame1, std::vector<KeyPoint>& keypoints_1, Mat& frame2, std::vector<KeyPoint>& keypoints_2,
                            std::vector<DMatch>&  good_matches);
    void MatchPointsFLANN(std::vector<DMatch>& good_matches, Mat& descriptors_1, Mat& descriptors_2 );
    void RollCorrectTransform(Matrix3d& cMat, double roll);
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

    ~GrassOdometer();
  private:
    Robot _robot = Robot::Misti();
    ColorRange _colors;
    int _numKeyPoints;
    Mat _lastFrame;
    std::vector<KeyPoint> _previousKeyPoints;
    Mat _lastFrameDescriptors;
    MatrixXd _lastFramePositions;
    bool _firstFrame;




};

#endif //GRASSODOMETER_H

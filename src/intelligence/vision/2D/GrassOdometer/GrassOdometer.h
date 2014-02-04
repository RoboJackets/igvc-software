#ifndef GRASSODOMETER_H
#define GRASSODOMETER_H

#include <list>
#include <eigen3/Eigen/Dense>

#include <common/events/Event.hpp>
#include <common/utils/ColorRange.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <common/datastructures/VisOdomData.hpp>
#include <hardware/sensors/camera/CameraInfo.h>
#include <common/datastructures/ImageData.hpp>
#include <common/utils/ImageUtils.h>


using namespace cv;
using namespace Eigen;
using namespace IGVC;

class RobotPosition;

class GrassOdometer
{
  public:
    GrassOdometer(ColorRange limits, int numKeyPoints=100);
    void processImage(ImageData src);
    void RemoveNonGrassPts(Mat& frame, std::vector<KeyPoint>& keypoints);
    void ShowCorrespondence(Mat& frame1, std::vector<KeyPoint>& keypoints_1, Mat& frame2, std::vector<KeyPoint>& keypoints_2,
                            std::vector<DMatch>&  good_matches);
    void MatchPointsFLANN(std::vector<DMatch>& good_matches, Mat& descriptors_1, Mat& descriptors_2 );
    Event<VisOdomData> onNewData;


    ~GrassOdometer();
  private:
    IGVC::CameraInfo _cam;
    ColorRange _colors;
    int _numKeyPoints;
    ImageData _lastFrame;
    std::vector<KeyPoint> _previousKeyPoints;
    Mat _lastFrameDescriptors;
    MatrixXd _lastFramePositions;
    bool _firstFrame;


    void findKeypointsSURF(Mat& frame, vector<KeyPoint>& theKeyPoints, Mat& theDescriptors, MatrixXd& thePositions);
    void findDeltas(Mat& newDescriptors, MatrixXd& newPos, double& deltax, double deltay);



};

#endif //GRASSODOMETER_H

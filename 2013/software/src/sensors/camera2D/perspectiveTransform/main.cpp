#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/lidar/NAV200.h>
#include <boost/thread.hpp>
#include <sensors/camera2D/laneDetection/LidarBasedObstacleHider.hpp>
#include <sensors/camera2D/perspectiveTransform/PerspectiveTransformer.hpp>
#include <sensors/camera3D/StereoImageRepeater.h>

#include "sensors/camera2D/laneDetection/CameraListener.hpp"

using namespace cv;
using namespace std;
using namespace IGVC::Sensors;

class PointCloudDisp
{
public:
    PointCloudDisp(CameraListener *camList)
    {

    }

private:
    void OnNewData()
};

int main()
{
    //NAV200 lidar;
    NAV200* lidarPointer = 0;

    //Bumblebee2 camera("/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");
    //Bumblebee2 camera;

    //StereoImageRepeater source("/home/robojackets/Desktop/img_right2.jpg", "/home/robojackets/Desktop/img_right2.jpg");
    StereoImageRepeater camera("/home/alex/Desktop/img_left2.jpg","/home/alex/Desktop/img_left2.jpg");
    //while(!camera.frameCount);

    namedWindow("Image");
    setMouseCallback("Image", OnMouse, 0);

    createTrackbar("t1", "Image", &thresh, 255, 0, 0);

    //CameraListener listener(&camera, &lidar);
    CameraListener listener(&camera, lidarPointer);

    waitKey(0);

    return 0;
}

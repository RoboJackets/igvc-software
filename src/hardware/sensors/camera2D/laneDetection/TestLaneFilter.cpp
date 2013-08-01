#include <sensors/lidar/NAV200.h>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/camera2D/laneDetection/CameraListener.hpp>
#include <iostream>
#include <sensors/camera3D/StereoPlayback.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace IGVC::Sensors;
using namespace std;

class CameraListenerListener
{
public:
    CameraListenerListener(CameraListener* camList)
        : _viewer("mapFrame"),
          LOnNewFileteredFrame(this),
          LOnNewMapFrame(this)
    {
        _camList = camList;
        _camList->OnNewFilteredFrame += &LOnNewFileteredFrame;
        _camList->OnNewData += &LOnNewMapFrame;
        _viewer.addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(1,0,0), "Line", 0);
    }

private:
    CameraListener* _camList;
    pcl::visualization::PCLVisualizer _viewer;

    void OnNewFileteredFrame(Mat frame)
    {
        imshow("Frame", frame);
    }
    LISTENER(CameraListenerListener, OnNewFileteredFrame, Mat);

    void OnNewMapFrame(pcl::PointCloud< pcl::PointXYZ > mapFrame)
    {
        _viewer.removeAllPointClouds(0);
        _viewer.addPointCloud(mapFrame.makeShared(), "map");
        _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "map");
        _viewer.spinOnce();

    }
    LISTENER(CameraListenerListener, OnNewMapFrame, pcl::PointCloud< pcl::PointXYZ>);
};

int main()
{
    namedWindow("Frame");

    cout << "setting up stereo source..." << endl;

    Bumblebee2 camera;

    //StereoPlayback camera("/home/robojackets/Desktop/camTesting/data/1CompCourse_left.mpeg", "/home/robojackets/Desktop/camTesting/data/1CompCourse_left.mpeg");

    cout << "Connecting to lidar..." << endl;

   //NAV200 lidar;

    cout << "Initializing listener..." << endl;

    CameraListener camList(&camera, 0);

    CameraListenerListener camListList(&camList);


    cout << "Running..." << endl;

    waitKey(0);

    cout << "done" << endl;

    return 0;
}

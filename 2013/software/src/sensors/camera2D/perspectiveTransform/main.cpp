#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/lidar/NAV200.h>
#include <boost/thread.hpp>

using namespace cv;
using namespace std;
using namespace IGVC::Sensors;

bool RUNNING = true;

class CameraListener
{
public:
    CameraListener(StereoSource *cam, Lidar *lidar)
        : LonNewFrame(this),
          LonNewLidarData(this)
    {
        cam->onNewData += &LonNewFrame;
        lidar->onNewData += &LonNewLidarData;
        FileStorage file("/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml", FileStorage::READ);
        file["mapMatrix"] >> transform;
        double metersPerPixel;
        file["mPerPixel"] >> metersPerPixel;
        file.release();
        pixelsPerM = 1.0 / metersPerPixel;
    }

private:
    void onNewFrame(StereoImageData data)
    {
        if(!RUNNING)
            return;
        Mat frame;

        resize(data.leftMat(), frame, Size(640,480));

        imshow("Raw", frame);

        Mat warped;

        warpPerspective(frame, warped, transform, Size(1280,960));

        resize(warped, warped, Size(640, 480));

//        {
//            int xL = warped.cols / 2;
//            int xR = warped.cols / 2;
//            while(xL > 0 && xL < warped.cols && xR > 0 && xR < warped.cols)
//            {
//                line(warped, Point2i(xL,0), Point2i(xL, warped.rows), Scalar(255,0,0));
//                line(warped, Point2i(xR,0), Point2i(xR, warped.rows), Scalar(255,0,0));
//                xL -= pixelsPerM;
//                xR += pixelsPerM;
//            }
//        }
//        {
//            for(int y = warped.rows; y >= 0; y -= pixelsPerM)
//            {
//                line(warped, Point2i(0, y), Point2i(warped.cols, y), Scalar(255,0,0));
//            }
//        }

        imshow("Warped", warped);

        _lock.lock();

        LidarPoint* points = lidarData.points;
        for(int i = 0; i < 1024; i++)
        {
            LidarPoint p = points[i];
            if(p.valid)
            {
                double distance = p.distance * pixelsPerM;
                int startX = distance * cos(p.angle) + warped.cols/2 + (0.1 * pixelsPerM);
                int startY = -1*(distance * sin(p.angle)) + warped.rows + (0.9 * pixelsPerM);
                int endX = startX + 580 * cos(p.angle);
                int endY = startY - 580 * sin(p.angle);
                line(warped, Point2i(startX, startY), Point2i(endX, endY), Scalar(0,0,0), 4);
            }
        }

        _lock.unlock();

        imshow("Image", warped);
    }
    LISTENER(CameraListener, onNewFrame, StereoImageData);

    void onNewLidarData(LidarState data)
    {
        if(!RUNNING)
            return;
        _lock.lock();
        lidarData = data;
        _lock.unlock();
    }
    LISTENER(CameraListener, onNewLidarData, LidarState);

    Mat transform;
    int pixelsPerM;
    LidarState lidarData;
    boost::mutex _lock;
};

int main()
{
    NAV200 lidar;

    Bumblebee2 camera("/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");

    while(!camera.frameCount);

    namedWindow("Image");

    CameraListener listener(&camera, &lidar);

    waitKey(0);

    RUNNING = false;

    return 0;
}

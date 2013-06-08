#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <sensors/camera3D/Bumblebee2.h>
#include <sensors/lidar/NAV200.h>
#include <boost/thread.hpp>
#include <sensors/camera2D/laneDetection/LidarBasedObstacleHider.hpp>
#include <sensors/camera2D/perspectiveTransform/PerspectiveTransformer.hpp>
#include <sensors/camera3D/StereoImageRepeater.h>

using namespace cv;
using namespace std;
using namespace IGVC::Sensors;

bool RUNNING = true;

Mat gFrame;

void OnMouse(int evtType, int x, int y, int, void*)
{
    if(evtType == EVENT_LBUTTONUP)
    {
        uchar* p = gFrame.ptr<uchar>(y);
        cout << (int)p[x*3] << " " << (int)p[x*3+1] << " " << (int)p[x*3+2] << endl;
    }
}

class CameraListener
{
public:
    CameraListener(StereoSource *cam, Lidar *lidar)
        : _transformer(cam),
          _hider(lidar, &_transformer.OnNewTransformedImage),
          LOnNewFrame(this)
    {
        _cam = cam;
        _lidar = lidar;
        _transformer.OnNewTransformedImage += &LOnNewFrame;
        _hider.OnNewProcessedFrame += &LOnNewFrame;
    }

private:
    Lidar* _lidar;
    StereoSource* _cam;
    PerspectiveTransformer _transformer;
    LidarBasedObstacleHider _hider;

    void OnNewFrame(Mat frame)
    {
        if(RUNNING)
        {
            blur(frame, frame, Size(9,9), Point(-1,-1));

            Mat HSV;
            cvtColor(frame, HSV, CV_BGR2HSV);
            vector<Mat> channels;
            split(HSV, channels);
            threshold(channels[0], channels[0], 90, 255, CV_THRESH_BINARY);
            threshold(channels[1], channels[1], 25, 255, CV_THRESH_BINARY);

            Mat output(frame.rows, frame.cols, CV_8UC1);
            bitwise_and(channels[0], channels[1], output);

            /*int erosion_size = 11;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size) );

            dilate(output, output, element);
            erode(output, output, element);*/

            int erosion_size = 6;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size) );

            erode(output, output, element);
            dilate(output, output, element);

            imshow("output", output);

            /*{ // Increase contrast of color image
                //http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
                Mat ycrcb;
                cvtColor(frame, ycrcb, CV_BGR2YCrCb);
                vector<Mat> channels;
                split(ycrcb, channels);
                equalizeHist(channels[0], channels[0]);
                merge(channels, ycrcb);
                cvtColor(ycrcb, frame, CV_YCrCb2BGR);
            }*/
            /*{
                uchar* p;
                for(int r = 0; r < frame.rows; r++)
                {
                    p = frame.ptr<uchar>(r);
                    for(int c = 0; c < frame.cols * frame.channels(); c+= 3)
                    {
                        if(abs(p[c+2] - p[c+1]) < 10 && p[c] < p[c+1] && p[c] < p[c+2])
                        {
                            p[c] = 0;
                            p[c+1] = 0;
                            p[c+2] = 255;
                        }
                    }
                }
            }*/

            gFrame = frame;
            imshow("Image", frame);
        }
    }
    LISTENER(CameraListener, OnNewFrame, Mat);
};

int main()
{
    //NAV200 lidar;

    //Bumblebee2 camera("/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");

    StereoImageRepeater source("/home/robojackets/Desktop/img_right2.jpg", "/home/robojackets/Desktop/img_right2.jpg");

    //while(!camera.frameCount);

    namedWindow("Image");
    setMouseCallback("Image", OnMouse, 0);

    CameraListener listener(&source, 0);-

    waitKey(0);

    RUNNING = false;

    return 0;
}

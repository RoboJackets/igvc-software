#ifndef CAMERALISTENER_H
#define CAMERALISTENER_H

#include <pcl/point_types.h>
#include "sensors/lidar/NAV200.h"
#include "sensors/camera3D/Bumblebee2.h"
#include "sensors/camera3D/StereoSource.hpp"
#include "common/Robot.h"
#include <sensors/camera2D/perspectiveTransform/PerspectiveTransformer.hpp>
#include <sensors/camera2D/laneDetection/LidarBasedObstacleHider.hpp>

#include "sensors/camera3D/StereoVidMaker.h"

bool RUNNING = true;
bool DEBUGGING = true;

using namespace IGVC::Sensors;

class CameraListener
{
public:
    CameraListener(StereoSource *cam, Lidar *lidar,
                   int intensityThresh=230,
                   string fileName = "/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml")
        : _transformer(cam),
          _hider(lidar, &_transformer.OnNewTransformedImage),
          _Writer(),
          LOnNewFrame(this)
    {
        _cam = cam;
        _lidar = lidar;
        //_transformer.OnNewTransformedImage += &LOnNewFrame;
        _hider.OnNewProcessedFrame += &LOnNewFrame;
        _robotInfo = Robot::Misti();

        _thresh = intensityThresh;

        FileStorage file(fileName, FileStorage::READ);
        file["mPerPixel"] >> _metersPerPixel;
        file.release();
        _Writer.open((const std::string&)"/home/robojackets/Desktop/lines.mpeg", CV_FOURCC('P','I','M','1'), 20, cv::Size(640,480), true);
    }

    Event<pcl::PointCloud< pcl::PointXYZ> > OnNewData;
    Event<Mat> OnNewFilteredFrame;

private:
    Lidar* _lidar;
    StereoSource* _cam;
    PerspectiveTransformer _transformer;
    LidarBasedObstacleHider _hider;
    int _thresh;
    Robot _robotInfo;
    VideoWriter _Writer;
    double _metersPerPixel;
    LISTENER(CameraListener, OnNewFrame, Mat);
    pcl::PointCloud<pcl::PointXYZ> _pointcloud;

    void OnNewFrame(Mat frame)
    {
        std::cout << "CamList newFrame" << std::endl;
        std::cout << "Running" << ( RUNNING ? "t" : "f" ) << std::endl;
        if(RUNNING)
        {
            if (DEBUGGING) {imshow("raw", frame);}

            _Writer << frame;

            blur(frame, frame, Size(9,9), Point(-1,-1));

            Mat HSV;
            cvtColor(frame, HSV, CV_BGR2HSV);
            vector<Mat> channels;
            split(HSV, channels);
            threshold(channels[0], channels[0], 90, 255, CV_THRESH_BINARY);
            threshold(channels[1], channels[1], 25, 255, CV_THRESH_BINARY);

            Mat output(frame.rows, frame.cols, CV_8UC1);
            bitwise_and(channels[0], channels[1], output);

            //imshow("H&S", output);

            Mat intensity;
            {
                vector<Mat> rgbChs;
                split(frame, rgbChs);
                intensity = ((rgbChs[0] + rgbChs[1]) + rgbChs[2])  / 3;
            }
            equalizeHist(intensity, intensity);

            //int intensityThresh = 230;
            threshold(intensity, intensity, _thresh, 255, CV_THRESH_BINARY);
            //imshow("Intense", intensity);

            bitwise_and(output, intensity, output);

            uchar* p;
            int x;
            int y;

            int nRows = output.rows;
            int nCols = output.cols;

            int centeredCol;


            for (int r=0;r<nRows;r++)
            {
              p = output.ptr<uchar>(r);
              for (int c=0;c<nCols*output.channels();c+=output.channels())
              {
                if (p[c] !=0)
                {
                  centeredCol = c - nCols/2;
                  x = _robotInfo.Dist2Front() + r*_metersPerPixel;
                  y = centeredCol*_metersPerPixel;

                  _pointcloud.points.push_back(pcl::PointXYZ(x, y, 0));

                }
              }
            }
            //_Writer << output;
            namedWindow("lines",1);
            imshow("lines", output);
            waitKey(10);
            OnNewFilteredFrame(output);
            OnNewData(_pointcloud);

            /*int erosion_size = 6;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size) );

            erode(output, output, element);
            dilate(output, output, element);*/

            /*
            imshow("output", output);
            gFrame = frame;
            imshow("Image", frame);
            */
        }
    }
};


#endif // CAMERALISTENER_H

#ifndef CAMERALISTENER_H
#define CAMERALISTENER_H

#include "sensors/lidar/NAV200.h"
#include "sensors/camera3D/Bumblebee2.h"
#include "sensors/camera3D/StereoSource.hpp"
#include "common/Robot.h"
#include <pcl/common/common_headers.h>

bool RUNNING = true;
bool DEBUGGING = false;

class CameraListener
{
public:
    CameraListener(StereoSource *cam, Lidar *lidar, int intensityThresh=230, string fileName = "/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml")
        : _transformer(cam),
          _hider(lidar, &_transformer.OnNewTransformedImage),
          LOnNewFrame(this)
    {
        _cam = cam;
        _lidar = lidar;
        _transformer.OnNewTransformedImage += &LOnNewFrame;
        _hider.OnNewProcessedFrame += &LOnNewFrame;
        _robotInfo = Robot::Misti();

        FileStorage file(fileName, FileStorage::READ);
        file["mPerPixel"] >> _metersPerPixel;
        file.release();
    }

private:
    Lidar* _lidar;
    StereoSource* _cam;
    PerspectiveTransformer _transformer;
    LidarBasedObstacleHider _hider;
    int _thresh;
    Robot _robotInfo;
    double _metersPerPixel;
    LISTENER(CameraListener, OnNewFrame, Mat);

    void OnNewFrame(Mat frame)
    {
        if(RUNNING)
        {
            if (DEBUGGING) {imshow("raw", frame);}

            blur(frame, frame, Size(9,9), Point(-1,-1));

            Mat HSV;
            cvtColor(frame, HSV, CV_BGR2HSV);
            vector<Mat> channels;
            split(HSV, channels);
            threshold(channels[0], channels[0], 90, 255, CV_THRESH_BINARY);
            threshold(channels[1], channels[1], 25, 255, CV_THRESH_BINARY);

            Mat output(frame.rows, frame.cols, CV_8UC1);
            bitwise_and(channels[0], channels[1], output);

            Mat intensity;
            {
                vector<Mat> rgbChs;
                split(frame, rgbChs);
                intensity = ((rgbChs[0] + rgbChs[1]) + rgbChs[2])  / 3;
            }
            equalizeHist(intensity, intensity);

            //int intensityThresh = 230;
            threshold(intensity, intensity, _thresh, 255, CV_THRESH_BINARY);
            imshow("Intense", intensity);

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

                }
              }
            }

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

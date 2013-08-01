#ifndef LIDARBASEDOBSTACLEHIDER_HPP_INCLUDED
#define LIDARBASEDOBSTACLEHIDER_HPP_INCLUDED

#include <sensors/lidar/Lidar.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <events/Event.hpp>

using namespace IGVC::Sensors;
using namespace cv;

class LidarBasedObstacleHider
{
public:
    LidarBasedObstacleHider(Lidar* lidar, Event<Mat>* source)
       : LOnNewLidar(this),
         LOnNewFrame(this)
    {
        FileStorage file("/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml", FileStorage::READ);
        file["mPerPixel"] >> _pixelsPerM;
        _pixelsPerM = 1.0/_pixelsPerM;
        file.release();

        if(lidar)
            lidar->onNewData += &LOnNewLidar;
        (*source) += &LOnNewFrame;
    }
    Event<Mat> OnNewProcessedFrame;

private:
    void OnNewLidar(LidarState data)
    {
        ///_lock.lock();
        _lidarData = data;
        //_lock.unlock();
    }
    LISTENER(LidarBasedObstacleHider, OnNewLidar, LidarState);

    void OnNewFrame(Mat frame)
    {
        //_lock.lock();

        LidarPoint* points = _lidarData.points;
        for(int i = 0; i < 1024; i++)
        {
            LidarPoint p = points[i];
            if(p.valid)
            {
                double distance = p.distance * _pixelsPerM;
                int startX = distance * cos(p.angle) + frame.cols/2 + (0.1 * _pixelsPerM);
                int startY = -1*(distance * sin(p.angle)) + frame.rows + (0.9 * _pixelsPerM);
                if(startX >= 0 && startX < frame.cols && startY >= 0 && startY < frame.rows)
                {
                    int endX = startX + 580 * cos(p.angle);
                    int endY = startY - 580 * sin(p.angle);
                    line(frame, Point2i(startX, startY), Point2i(endX, endY), Scalar(0,0,0), 5);
                    line(frame, Point2i(startX, startY), Point2i(startX, 0), Scalar(0,0,0), 5);
                }
            }
        }
       // _lock.unlock();
        OnNewProcessedFrame(frame.clone());
    }
    LISTENER(LidarBasedObstacleHider, OnNewFrame, Mat);

    boost::mutex _lock;
    LidarState _lidarData;
    double _pixelsPerM;
};

#endif // LIDARBASEDOBSTACLEHIDER_HPP_INCLUDED

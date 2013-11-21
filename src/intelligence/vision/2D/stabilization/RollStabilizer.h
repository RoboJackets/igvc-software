#ifndef ROLLSTABILIZER_H
#define ROLLSTABILIZER_H

#include <opencv2/opencv.hpp>
#include <serial/ASIOSerialPort.h>
#include <boost/thread.hpp>
#include <events/Event.hpp>
#include <sensors/ardupilot/Ardupilot.hpp>

using namespace cv;

class RollStabilizer
{
    public:
        RollStabilizer(VideoCapture* camera, Ardupilot* imu);
        virtual ~RollStabilizer();
        Event<Mat> OnNewFrame;

    private:
        VideoCapture* _cam;
        Ardupilot* _imu;
        double _roll;
        bool _running;
        boost::thread _imgThread;
        void img_thread_run();
        void onNewRawIMU(IMURawData data);
        LISTENER(RollStabilizer, onNewRawIMU, IMURawData);
};

#endif // ROLLSTABILIZER_H

#ifndef STEREOIMAGEREPEATER_H
#define STEREOIMAGEREPEATER_H

#include <hardware/sensors/camera/StereoSource.hpp>
#include <boost/thread.hpp>

class StereoImageRepeater : public StereoSource
{
    public:
        StereoImageRepeater(std::string pathLeft, std::string pathRight);
        virtual ~StereoImageRepeater();
        virtual bool IsConnected();
    protected:
    private:
        cv::Mat _left;
        cv::Mat _right;
        boost::thread _thread;
        void thread_run();
        float _fps;
};

#endif // STEREOIMAGEREPEATER_H

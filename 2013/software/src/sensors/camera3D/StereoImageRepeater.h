#ifndef STEREOIMAGEREPEATER_H
#define STEREOIMAGEREPEATER_H

#include <sensors/camera3D/StereoSource.hpp>
#include <boost/thread.hpp>

class StereoImageRepeater : public StereoSource
{
    public:
        StereoImageRepeater(std::string pathLeft, std::string pathRight);
        virtual ~StereoImageRepeater();
    protected:
    private:
        Mat _left;
        Mat _right;
        boost::thread _thread;
        void thread_run();
};

#endif // STEREOIMAGEREPEATER_H

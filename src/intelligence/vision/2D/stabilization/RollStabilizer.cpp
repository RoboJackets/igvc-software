#include "RollStabilizer.h"

RollStabilizer::RollStabilizer(VideoCapture* camera, Ardupilot* imu)
    : LonNewRawIMU(this)
{
    _cam = camera;
    _imu = imu;
    _imu->onNewRawData += &LonNewRawIMU;
    _roll = 0;
    _running = true;
    _imgThread = boost::thread(boost::bind(&RollStabilizer::img_thread_run, this));
}

void RollStabilizer::img_thread_run()
{
    Mat frame, rotated;
    while(_running && _cam)
    {
        (*_cam) >> frame;

        Mat rot_mat = Mat(2,3,CV_32FC1);
        CvPoint2D32f center = cvPoint2D32f(frame.cols/2, frame.rows/2);
        rot_mat = getRotationMatrix2D(center, ( -_roll / M_PI ) * 180.0, 1);
        warpAffine(frame, rotated, rot_mat, cvSize(rotated.cols, rotated.rows));

        OnNewFrame(rotated);
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    cout << "Camera thread finished." << endl;
}

void RollStabilizer::onNewRawIMU(IMURawData data)
{
    _roll = data.roll;
}

RollStabilizer::~RollStabilizer()
{
    _running = false;
    //_imgThread.interrupt();
    _imgThread.join();
    cout << "join returned." << endl;
    _imu->onNewRawData -= &LonNewRawIMU;
}

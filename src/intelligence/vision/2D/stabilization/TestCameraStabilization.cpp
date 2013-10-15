#include <opencv2/opencv.hpp>
#include <iostream>
#include <serial/ASIOSerialPort.h>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <boost/thread.hpp>
#include <sensors/camera2D/stabilization/RollStabilizer.h>
#include <fstream>

using namespace cv;
using namespace std;

/*class CameraStabilizer
{
private:
    VideoCapture _cam;
    ASIOSerialPort _imu;
    double _roll;
    Mat _frame, _rotated;
    boost::thread _imgThread;
public:
    CameraStabilizer()
     : _cam(1),
       _imu("/dev/ttyIMU", 115200),
       LOnSerialLine(this)
    {
        if(!_cam.isOpened())
        {
            cerr << "Could not open USB camera." << endl;
            exit(-1);
        }
        _imu.onNewLine += &LOnSerialLine;
        _imu.startEvents();
        _roll = 0;
        _imgThread = boost::thread(boost::bind(&CameraStabilizer::imgThreadRun, this));
    }


    void OnSerialLine(string line)
    {
        if(line.c_str()[0] == 'A')
        {
            string val = line.substr(2, line.size() - 2);
            _roll = atof(val.c_str());
            cout << _roll << endl;
        }
    }
    LISTENER(CameraStabilizer, OnSerialLine, string);

    void imgThreadRun()
    {
        while(true)
        {
            _cam >> _frame;


            Mat rot_mat = Mat(2,3,CV_32FC1);
            CvPoint2D32f center = cvPoint2D32f(_frame.cols/2, _frame.rows/2);
            rot_mat = getRotationMatrix2D(center, ( -_roll / M_PI ) * 180.0, 1);

            warpAffine(_frame, _rotated, rot_mat, cvSize(_rotated.cols, _rotated.rows));

            imshow("Preview", _rotated);
            if(waitKey(10) >= 0)
                exit(0);
        }
    }

};*/

bool running = true;

class StabilizerListener
{
private:
    RollStabilizer *_stabilizer;
public:
    StabilizerListener(RollStabilizer *stabilizer)
        : LOnNewStabilizedFrame(this)
    {
        _stabilizer = stabilizer;
        _stabilizer->OnNewFrame += &LOnNewStabilizedFrame;
    }

    void OnNewStabilizedFrame(Mat frame)
    {
        if(running)
        imshow("Preview", frame);
    }
    LISTENER(StabilizerListener, OnNewStabilizedFrame, Mat);

    ~StabilizerListener()
    {
        _stabilizer->OnNewFrame -= &LOnNewStabilizedFrame;
        //_stabilizer = 0;
    }
};

int main() {

    std::streambuf *coutbuf = std::cout.rdbuf();
    std::ofstream out("log.txt");
    std::cout.rdbuf(out.rdbuf());

    namedWindow("Preview", 1);

    VideoCapture cam(1);
    if(!cam.isOpened())
    {
        cerr << "Couldn't open camera." << endl;
        return -1;
    }

    //ASIOSerialPort imu("/dev/ttyIMU", 115200);
    //imu.startEvents();
    Ardupilot imu;

    RollStabilizer stabilizer(&cam, &imu);

    StabilizerListener listener(&stabilizer);

    while(true)
    {
        if(waitKey(10) >= 0)
        {
            cout << "Main thread breaking...." << endl;
            break;
        }
    }

    running = false;

    cout << "Main thread returning..." << endl;

    std::cout.rdbuf(coutbuf);

    return 0;


    /*CameraStabilizer camStab;

    while(true);

    return 0;*/
}

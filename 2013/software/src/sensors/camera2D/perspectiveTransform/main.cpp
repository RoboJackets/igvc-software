#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

int main()
{
    Mat frame, map_matrix, transformed;
    VideoCapture cam(1);
    if(!cam.isOpened())
    {
        cerr << "Could not open camera." << endl;
        return 1;
    }

    Point2f src_pt[4], dst_pt[4];

    src_pt[0] = Point2f(119.0,267.0);
    src_pt[1] = Point2f(  0.0,470.0);
    src_pt[2] = Point2f(639.0,470.0);
    src_pt[3] = Point2f(522.0,267.0);

    dst_pt[0] = Point2f(320.0,420.0);
    dst_pt[1] = Point2f(320.0,720.0);
    dst_pt[2] = Point2f(960.0,720.0);
    dst_pt[3] = Point2f(960.0,420.0);

    while(true)
    {
        cam >> frame;

        imshow("raw", frame);

        map_matrix = getPerspectiveTransform(src_pt, dst_pt);

        warpPerspective(frame, transformed, map_matrix, Size(1280, 720));

        resize(transformed, transformed, Size(640,480), 0, 0, CV_INTER_AREA);

        imshow("transformed", transformed);

        {
            char in = waitKey(10);
            if(in == ' ')
            {
                stringstream name;
                name << "/home/matt/Pictures/";
                time_t t;
                time(&t);
                name << t << "_trans.jpg";
                imwrite(name.str(), transformed);
                name.str("");
                name.clear();
                name << "/home/matt/Pictures/";
                name << t << "_raw.jpg";
                imwrite(name.str(), frame);
            }
            else if(in >= 0)
            {
                break;
            }
        }
    }
}

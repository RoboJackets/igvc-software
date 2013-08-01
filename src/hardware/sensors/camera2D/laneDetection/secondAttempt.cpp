#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

int main()
{
//    Mat frame, map_matrix, transformed;
//    VideoCapture cam(1);
//    if(!cam.isOpened())
//    {
//        cerr << "Could not open camera." << endl;
//        return 1;
//    }
//
//    Point2f src_pt[4], dst_pt[4];
//    {
//        src_pt[0] = Point2f(119.0,267.0);
//        src_pt[1] = Point2f(  0.0,470.0);
//        src_pt[2] = Point2f(639.0,470.0);
//        src_pt[3] = Point2f(522.0,267.0);
//
//        dst_pt[0] = Point2f(320.0,420.0);
//        dst_pt[1] = Point2f(320.0,720.0);
//        dst_pt[2] = Point2f(960.0,720.0);
//        dst_pt[3] = Point2f(960.0,420.0);
//    }
//
//    imshow("raw", frame);
//
//    map_matrix = getPerspectiveTransform(src_pt, dst_pt);
//
//    warpPerspective(frame, transformed, map_matrix, Size(1280, 720));
//
//    resize(transformed, transformed, Size(640,480), 0, 0, CV_INTER_AREA);
//
//    imshow("transformed", transformed);



    Mat frame = imread("/home/matt/Pictures/1369630183_trans.jpg");
    imshow("Raw", frame);

    { // Increase contrast of color image
        //http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
        Mat ycrcb;
        cvtColor(frame, ycrcb, CV_BGR2YCrCb);
        vector<Mat> channels;
        split(ycrcb, channels);
        equalizeHist(channels[0], channels[0]);
        merge(channels, ycrcb);
        cvtColor(ycrcb, frame, CV_YCrCb2BGR);
    }

    {
        uchar *p;
        for(int r = 0; r < frame.rows; r++)
        {
            p = frame.ptr<uchar>(r);
            for(int c = 0; c < frame.cols * frame.channels(); c += 3)
            {
                if(abs(p[c] - p[c+1]) < 30 && abs(p[c] - p[c+2]) < 30 && p[c] > 225)
                {
                    p[c] = 255;
                    p[c+1] = 255;
                    p[c+2] = 255;
                }
                else
                {
                    p[c] = 0;
                    p[c+1] = 0;
                    p[c+2] = 0;
                }
            }
        }
    }

    int erosion_size = 5;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );

    erode(frame, frame, element);
    dilate(frame, frame, element);

    //cvtColor(frame, frame, CV_BGR2GRAY);

    //Canny(frame, frame, 100, 100);

    imshow("Gray", frame);

    waitKey(0);
}

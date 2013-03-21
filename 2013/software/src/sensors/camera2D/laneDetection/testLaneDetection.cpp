#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;

Mat frame, filtered, binary;

void on_mouse(int evt, int c, int r, int flags, void* param)
{
    if(evt==CV_EVENT_LBUTTONDOWN)
    {
        uchar* row = filtered.ptr<uchar>(r);
        int blue = row[c*3];
        int green = row[c*3+1];
        int red = row[c*3+2];
        std::cout << red << " " << green << " " << blue << std::endl;
    }
}

int main()
{
    VideoCapture cam(1);

    namedWindow("Filtered");
    setMouseCallback("Filtered", on_mouse, 0);

    while(true)
    {
        cam >> frame;

//        frame = imread("/home/matt/Pictures/DSC09490.JPG", 1);

        resize(frame, frame, Size(640,480));

        Rect roi(0, 200, frame.cols, frame.rows -200 );

//        frame = frame(roi);

        frame *= 2;

        imshow("Orig", frame);

        boxFilter(frame, filtered,0,Size(9,9));

        uchar* p;
        for(int i = 0; i < filtered.rows; i++)
        {
            p = filtered.ptr<uchar>(i);
            for(int j = 0; j < filtered.cols*filtered.channels(); j += 3)
            {
                uchar blue = p[j];
                uchar green = p[j+1];
                uchar red = p[j+2];

                double sum = red+ green + blue;

                if(sum > 0)
                {
                    p[j] = blue / sum * 255;
                    p[j+1] = green / sum * 255;
                    p[j+2] = red / sum * 255;
                }
            }
        }

        imshow("Filtered", filtered);

        binary = filtered.clone();
        binary = Mat(filtered.rows, filtered.cols, CV_8UC3);
        filtered.copyTo(binary);

        uchar* binary_p;
        for(int i = 0; i < binary.rows; i++)
        {

            binary_p = binary.ptr<uchar>(i);
            for(int j = 0; j < binary.cols*binary.channels(); j += 3)
            {
                uchar blue = binary_p[j];
                uchar green = binary_p[j+1];
                uchar red = binary_p[j+2];
                if(abs(blue-green) < 7 && abs(red-green) < 7 && abs(red-blue) < 7)
                {
                    binary_p[j] = 255;
                    binary_p[j+1] = 255;
                    binary_p[j+2] = 255;
                } else {
                    binary_p[j] = 0;
                    binary_p[j+1] = 0;
                    binary_p[j+2] = 0;
                }
            }
        }

        cvtColor(binary, binary, CV_BGR2GRAY);

        imshow("binary", binary);

        vector<Vec4i> lines;
        HoughLinesP(binary, lines, 1, CV_PI/180, 50, 50, 10 );
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            Point p0(l[0], l[1]);
            Point p1(l[2], l[3]);
            double dx = (double)p1.x - (double)p0.x;
            double m = dx != 0 ? ((double)p1.y - (double)p0.y) / dx : 1000000000.0;
            m = m < 0 ? m * -1 : m;
            if(m > 0.2)
            {
                line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
            }
        }

        imshow("Lines", frame);

        if(waitKey(10) >= 0)
            break;
    }

    return 0;
}

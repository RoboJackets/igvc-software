#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <set>
#include <vector>

using namespace cv;
using namespace std;

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

void normalizeRGB(Mat mat)
{
    uchar* p;
    for(int i = 0; i < mat.rows; i++)
    {
        p = mat.ptr<uchar>(i);
        for(int j = 0; j < mat.cols*mat.channels(); j += 3)
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
}

int main()
{
    int trackVal = 215;
    cvNamedWindow("Binary");
    cvCreateTrackbar("trackbar", "Binary", &trackVal, 255);
    while(true) {
    frame = imread("/home/matt/Pictures/2005IGVCcoursePics 154.jpg", 1);
    resize(frame, frame, Size(640,480));
    imshow("Raw", frame);

    frame.copyTo(filtered);

//    boxFilter(filtered, filtered, 0, Size(9,9));

    cvtColor(filtered, filtered, CV_BGR2HSV);

    {
        vector<Mat> channels;
        split(filtered, channels);
        Mat H = channels[0];
        equalizeHist(H, H);
        imshow("Hue", H);
        Mat S = channels[1];
        S.at
    }

    imshow("Filtered", filtered);

/*    { // Increase contrast of color image
        // http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
        Mat ycrcb;
        cvtColor(filtered, ycrcb, CV_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb, channels);

        equalizeHist(channels[0], channels[0]);

        merge(channels, ycrcb);

        cvtColor(ycrcb, filtered, CV_YCrCb2BGR);
    }

    imshow("Filtered", filtered);

    binary = Mat(filtered.rows, filtered.cols, CV_8UC3);
    filtered.copyTo(binary);

    uchar* p;
    for(int r = 0; r < binary.rows; r++)
    {
        p = binary.ptr<uchar>(r);
        for(int c = 0; c < binary.cols*binary.channels(); c+=3)
        {
            uchar blue = p[c];
            uchar green = p[c+1];
            uchar red = p[c+2];

            p[c] = 0;
            p[c+1] = 0;
            p[c+2] = 0;

//            if(abs(blue - 85) < 10 && abs(green - 85) < 10 && abs(red - 85) < 10) //White
            if(blue > trackVal && green > trackVal && red > trackVal)
            {
                p[c] = 255;
                p[c+1] = 255;
                p[c+2] = 255;
            }
        }
    }

    cvtColor(binary, binary, CV_BGR2GRAY);

    imshow("Binary", binary);*/

    if(waitKey(10) > 0)
        break;
    }

    return 0;
}

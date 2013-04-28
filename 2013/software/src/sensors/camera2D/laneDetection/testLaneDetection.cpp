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
    //cvNamedWindow("Binary");
    //cvCreateTrackbar("trackbar", "Binary", &trackVal, 255);
    while(true) {
    frame = imread("/home/matt/Pictures/2005IGVCcoursePics 154.jpg", 1);
    resize(frame, frame, Size(640,480));

    { // ROI
        int startrow = frame.rows / 4;
        frame = frame(Rect(0,startrow,frame.cols,frame.rows - startrow));
    }

    imshow("Raw", frame);

    frame.copyTo(filtered);

    boxFilter(filtered, filtered, 0, Size(9,9));

    { // Filter Orange hues to Black
        Mat HSV;
        cvtColor(filtered, HSV, CV_BGR2HSV);
        vector<Mat> channels;
        split(HSV, channels);
        Mat H = channels[0];
        Mat V = channels[2];
        uchar* pH;
        uchar* pV;
        for(int r = 0; r < H.rows; r++)
        {
            pH = H.ptr<uchar>(r);
            pV = V.ptr<uchar>(r);
            for(int c = 0; c < H.cols; c++)
            {
                uchar hVal = pH[c];
                if(abs(hVal - 170) < 10 || abs(hVal - 0) < 10)
                {
                    pV[c] = 0;
                }
            }
        }
        //imshow("H", channels[0]);
        merge(channels, filtered);
        cvtColor(filtered, filtered, CV_HSV2BGR);
    }

    { // Remove White segments of columns between Orange segments
        Mat transposed;
        transpose(filtered, transposed);
        uchar* p;
        bool seenBlack, seenWhite;
        int startInd = 0;
        for(int c = 0; c < transposed.rows; c++)
        {
            p = transposed.ptr<uchar>(c);
            seenBlack = false;
            seenWhite = false;
            for(int r = 0; r < transposed.cols*transposed.channels(); r+=3)
            {
                uchar blue = p[r], green = p[r+1], red = p[r+2];
                if( red == 0 && green == 0 && blue == 0)
                {
                    if(seenBlack && seenWhite)
                    {
                        for(int i = startInd; i <= r; i++)
                        {
                            p[i] = 0;
                            p[i+1] = 0;
                            p[i+2] = 0;
                        }
                    }
                    else if(!seenBlack)
                    {
                        seenBlack = true;
                        startInd = r;
                    }
                } else if(abs(red-green) < 5 && abs(green-blue) < 30)//if(red > 150 && green > 150 && blue > 150)
                {
                    if(seenBlack && !seenWhite)
                    {
                        seenWhite = true;
                    }
                }
            }
        }
        //imshow("Transposed", transposed);
        transpose(transposed, filtered);
    }

    imshow("Filtered", filtered);
    //cvSetMouseCallback("Raw", on_mouse, 0);

    cvtColor(filtered, filtered, CV_BGR2GRAY);
    threshold(filtered, filtered, 0, 1, THRESH_BINARY);
    cvtColor(filtered, filtered, CV_GRAY2BGR);
    Mat masked;
    multiply(filtered, frame, masked);

    { // Increase contrast of color image
        // http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
        Mat ycrcb;
        cvtColor(masked, ycrcb, CV_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb, channels);

        equalizeHist(channels[0], channels[0]);

        merge(channels, ycrcb);

        cvtColor(ycrcb, masked, CV_YCrCb2BGR);
    }
    imshow("Masked", masked);

    binary = Mat(masked.rows, masked.cols, CV_8UC3);
    masked.copyTo(binary);

    { // Threshold for White
        uchar threshold = 240;
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

                if(blue > threshold && green > threshold && red > threshold)
                {
                    p[c] = 255;
                    p[c+1] = 255;
                    p[c+2] = 255;
                }
            }
        }

        cvtColor(binary, binary, CV_BGR2GRAY);
    }


    int erosion_size =2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
                          cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                          cv::Point(erosion_size, erosion_size) );

    erode(binary, binary, element);
    dilate(binary, binary, element);

    imshow("Binary", binary);

    if(waitKey(10) > 0)
        break;
    }

    return 0;
}

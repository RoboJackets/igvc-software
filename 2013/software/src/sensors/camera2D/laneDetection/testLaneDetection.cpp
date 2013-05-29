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
    Mat* img = (Mat*)param;
    if(evt==CV_EVENT_LBUTTONDOWN)
    {
        uchar* row = img->ptr<uchar>(r);
        int a = row[c*3];
        int b = row[c*3+1];
        int c = row[c*3+2];
        std::cout << a << "  " << b << "  " << c << std::endl;
    }
    /*if(evt==CV_EVENT_LBUTTONDOWN)
    {
        uchar* row = filtered.ptr<uchar>(r);
        int blue = row[c*3];
        int green = row[c*3+1];
        int red = row[c*3+2];
        std::cout << red << " " << green << " " << blue << std::endl;
    }*/
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
    Mat frame = imread("/home/matt/Pictures/CalStateData.png");

    imshow("raw", frame);

    blur(frame, frame, Size(3, 3), Point(-1, -1) );

    Mat grassfiltered;
    {
        vector<Mat> channels;
        split(frame, channels);
        channels[0] = channels[0] - 0.5 * channels[1];

        merge(channels, grassfiltered);
    }

    cvtColor(grassfiltered, grassfiltered, CV_BGR2GRAY);

    threshold(grassfiltered, grassfiltered, 200, 255, CV_THRESH_BINARY);

    imshow("Grass filter", grassfiltered);

    Mat obstacles(frame.rows, frame.cols, CV_8UC3);

    frame.copyTo(obstacles);

    {
        Mat HSV;
        cvtColor(obstacles, HSV, CV_BGR2HSV);
        vector<Mat> channels;
        split(HSV, channels);
        channels[2] = Mat(HSV.size(), CV_8UC1, Scalar(200));
        channels[1] *= 2.0;
        merge(channels, HSV);
        cvtColor(HSV, obstacles, CV_HSV2BGR);
    }

    {
        uchar *p;
        for(int r = 0; r < obstacles.rows; r++)
        {
            p = obstacles.ptr<uchar>(r);
            for(int c = 0; c < obstacles.cols * obstacles.channels(); c += 3)
            {
                bool isNotGreenOrWhite = false;

                if(p[c] < p[c+1] - 10 && p[c+2] < p[c+1] - 10)
                {
                    isNotGreenOrWhite = true;
                }

                if(p[c] > 190 && p[c+1] > 190 && p[c+2] > 190)
                {
                    isNotGreenOrWhite = true;
                }

                if(isNotGreenOrWhite)
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

    erode(obstacles, obstacles, element);
    dilate(obstacles, obstacles, element);

    Mat mask;
    {
        Mat frameT = frame.t();
        Mat obstT = obstacles.t();
        flip(frameT, frameT, 1);
        flip(obstT, obstT, 1);
        uchar *p;
        uchar* op;
        for(int r = 0; r < frameT.rows; r++)
        {
            bool blackout = false;
            p = frameT.ptr<uchar>(r);
            op = obstT.ptr<uchar>(r);
            for(int c = 0; c < frameT.cols * frameT.channels(); c += 3)
            {
                if(blackout)
                {
                    p[c] = 0;
                    p[c+1] = 0;
                    p[c+2] = 0;
                }
                else
                {
                    if(op[c]+op[c+1]+op[c+2] > 0)
                    {
                        p[c] = 0;
                        p[c+1] = 0;
                        p[c+2] = 0;
                        blackout = true;
                    }
                    else
                    {
                        p[c] = 1;
                        p[c+1] = 1;
                        p[c+2] = 1;
                    }
                }
            }
        }
        flip(frameT, frameT, 1);
        flip(obstT, obstT, 1);
        mask = frameT.t();
        imshow("blocked out", mask * 255);
    }

    imshow("obst", obstacles);

    cvtColor(mask, mask, CV_BGR2GRAY);

    Mat lines = mask.mul(grassfiltered);

    imshow("lines", lines);

    waitKey(0);
    return 0;
}

int main3()
{
    while(true) {

        namedWindow("Frame");
        int val1;
        int val2;
        cvCreateTrackbar("trackbar1", "Frame", &val1, 1000);
        cvCreateTrackbar("trackbar2", "Frame", &val2, 1000);

        frame = imread("/home/matt/Pictures/IMG_0817.JPG", 1);
        flip(frame, frame, 0);
        flip(frame, frame, 1);
        resize(frame, frame, Size(640,480));

        cvtColor(frame, frame, CV_BGR2GRAY);

        Canny(frame, frame, val1, val2);

        imshow("Frame", frame);

        if(waitKey(10) >= 0)
            break;
    }
    return 0;
}

int main2()
{
    int trackVal = 215;
    //cvNamedWindow("Binary");
    //cvCreateTrackbar("trackbar", "Binary", &trackVal, 255);
    while(true) {

        frame = imread("/home/matt/Pictures/IMG_0817.JPG", 1);
        flip(frame, frame, 0);
        flip(frame, frame, 1);
        /*resize(frame, frame, Size(640,480));

        boxFilter(frame, frame, 0, Size(9, 9));

        imshow("Orig", frame);

        {
            uchar *p;
            uchar* p2;
            for(int r = 0; r < frame.rows; r++)
            {
                p = frame.ptr<uchar>(r);
                if(r+1 < frame.rows)
                    p2 = frame.ptr<uchar>(r+1);
                for(int c = 0; c < frame.cols * frame.channels(); c+=frame.channels())
                {
                    int diffBG = p[c] - p[c+1];
                    int diffGR = p[c+1] - p[c+2];
                    int avdiff = (diffBG + diffGR) / 2;
                    bool IAmWhite = avdiff < 20 && p[c] > 220;
                    bool MyNeighboorIsWhite = true;
                    if(r+1 < frame.rows)
                    {
                        diffBG = p2[c] - p2[c+1];
                        diffGR = p2[c+1] - p2[c+2];
                        avdiff = (diffBG + diffGR) / 2;
                        MyNeighboorIsWhite = avdiff < 20 && p2[c] > 220;
                    }
                    if(IAmWhite && !MyNeighboorIsWhite)
                    {
                        p[c] = 255;
                        p[c+1] = 255;
                        p[c+2] = 255;
                    } else {
                        p[c] = 0;
                        p[c+1] = 0;
                        p[c+2] = 0;
                    }
                }
            }
        }

        cvtColor(frame, frame, CV_BGR2GRAY);
        //dilate(binary, binary, element);

        imshow("Frame", frame);*/

        //frame = imread("/home/matt/Pictures/2005IGVCcoursePics 154.jpg", 1);
//        frame = imread("/home/matt/Pictures/2005IGVCcoursePics 089.jpg", 1);
        resize(frame, frame, Size(640,480));

        /*{ // ROI
            int startrow = frame.rows / 4;
            frame = frame(Rect(0,startrow,frame.cols,frame.rows - startrow));
        }*/

        imshow("Raw", frame);

        frame.copyTo(filtered);

        boxFilter(filtered, filtered, 0, Size(9, 9));



        /*{
            Mat lab;
            cvtColor(filtered, lab, CV_BGR2Lab);
            uchar* row;
            for(int r = 0; r < lab.rows; r++)
            {
                row = lab.ptr<uchar>(r);
                for(int c = 0; c < lab.cols * lab.channels(); c++)
                {
                    uchar L = row[c], a = row[c+1], b = row[c+2];
                    int da = a - 115;
                    da = (da < 0) ? -1*da : da;
                    int db = b - 160;
                    db = (db < 0) ? -1*db : db;
                    if(da < 10 && db < 20)
                    {
                        row[c] = 0;
                    }
                }
            }
            cvtColor(lab, filtered, CV_Lab2BGR);
        }*/


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
            imshow("H", channels[0]);
            merge(channels, filtered);
            cvtColor(filtered, filtered, CV_HSV2BGR);
        }

        imshow("Filtered", filtered);
//        cvSetMouseCallback("Filtered", on_mouse, 0);

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
            imshow("Transposed", transposed);
            transpose(transposed, filtered);
        }
        imshow("Filtered", filtered);
//        cvSetMouseCallback("Raw", on_mouse, 0);

        cvtColor(filtered, filtered, CV_BGR2GRAY);
        threshold(filtered, filtered, 0, 1, THRESH_BINARY);
        cvtColor(filtered, filtered, CV_GRAY2BGR);
        Mat masked;
        multiply(filtered, frame, masked);

        boxFilter(masked, masked, 0, Size(9,9));

        { // Increase contrast of color image
            //http://stackoverflow.com/questions/15007304/histogram-equalization-not-working-on-color-image-opencv
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

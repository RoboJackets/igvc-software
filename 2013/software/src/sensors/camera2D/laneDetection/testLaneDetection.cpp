#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <set>
#include <vector>
//#include <sensors/camera2D/laneDetection/LaneMapper.hpp>
#include <sensors/DataStructures/StereoImageData.hpp>
#include <common/utils/ImageUtils.h>
#include <fstream>

using namespace cv;
using namespace std;

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

    vector<KeyPoint> keyPoints;

    {
        uchar* p;
        for(int r = 0; r < lines.rows; r++)
        {
            p = lines.ptr<uchar>(r);
            for(int c = 0; c < lines.cols; c++)
            {
                if(p[c] == 255)
                {
                    // Add to vector.
                    keyPoints.push_back(KeyPoint(c, r, 1));
                }
            }
        }
    }
    MatrixXd pos;
    Robot robot = Robot::Misti();
    CameraInfo cameraInfo = CameraInfo::Bumblebee2_BB2_08S2C_38();
    computeOffsets(keyPoints, pos, robot, cameraInfo, lines.rows, lines.cols);

    ofstream output;
    output.open("LaneDetectionOutput.txt");
    for(int c = 0; c < countNonZero(lines); c++)
    {
        for(int r = 0; r < 4; r++)
            output << pos(r, c) << ",";
        output << endl;
    }

    output << pos;
    output.close();

    waitKey(0);
    return 0;
}

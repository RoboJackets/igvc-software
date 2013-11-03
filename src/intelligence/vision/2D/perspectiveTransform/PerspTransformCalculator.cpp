#include <opencv2/opencv.hpp>
#include <sensors/camera3D/Bumblebee2.h>

#include <iostream>
using namespace std;
using namespace cv;

Mat frame;
Point2f srcPoints[4];
int numPoints = 0;
string windowName = "Perspective Transform Calculator";
bool clicksEnabled = true;

void onMouse(int event, int x, int y, int, void*)
{
    if(event != EVENT_LBUTTONDOWN)
        return;

    if(clicksEnabled)
    {
        int ind = numPoints % 4;

        srcPoints[ind] = Point(x, y);
        numPoints++;
        Mat tmp = frame.clone();
        for(int i = 0; i < numPoints; i++)
        {
            circle(tmp, srcPoints[i], 3, Scalar(0,0,255), 2);
        }
        imshow(windowName, tmp);
    }
}

int main()
{
    /*cout << "Initing..." << endl;
    Bumblebee2 camera("/home/robojackets/igvc/2013/software/src/sensors/camera3D/calib/out_camera_data.xml");

    cout << "Waiting..." << endl;

    while(!camera.frameCount);

    cout << "Looping..." << endl;

    frame = camera.left().mat();*/

    Mat map_matrix;
    double metersPerPixel;

    char response;
    do
    {
        numPoints = 0;
        clicksEnabled = true;

        frame = imread("/home/robojackets/Desktop/img_left0.jpg");
        //frame = camera.LeftMat();
        resize(frame, frame, Size(640,480));

        cout << "Please select 4 corners of your 1m x 1m floor square, then press any key to continue." << endl;

        namedWindow(windowName);
        setMouseCallback(windowName,onMouse, 0);
        imshow(windowName, frame);

        do
        {
            waitKey(0);
        } while(numPoints < 4);

        clicksEnabled = false;

        cout << endl << "Computing transformation...";
        cout.flush();

        Point2f UL = srcPoints[0], UR = srcPoints[1], LL = srcPoints[2], LR = srcPoints[3];

        while(UL.y > LL.y || UL.x > UR.x || UR.y > LR.y || LL.x > LR.x)
        {
            Point tmp;
            if(UL.y > LL.y)
            {
                tmp = LL;
                LL = UL;
                UL = tmp;
            }
            if(UL.x > UR.x)
            {
                tmp = UR;
                UR = UL;
                UL = tmp;
            }
            if(UR.y > LR.y)
            {
                tmp = UR;
                UR = LR;
                LR = tmp;
            }
            if(LL.x > LR.x)
            {
                tmp = LL;
                LL = LR;
                LR = tmp;
            }
        }

        srcPoints[0] = UL;
        srcPoints[1] = LL;
        srcPoints[2] = LR;
        srcPoints[3] = UR;

        int width = LR.x - LL.x;

        // UL, LL, LR, UR
        Point2f dstPoints[4] = {
            Point2f( LL.x, LL.y - width),
            LL,
            Point2f(LR.x, LL.y),
            Point2f( LR.x, LL.y - width)
        };

        Mat CheckPoints = frame.clone();
        for(int i = 0; i < 4; i++)
            circle(CheckPoints, srcPoints[i], 3, Scalar(0,0,255), 2);
        for(int i = 0; i < 4; i++)
            circle(CheckPoints, dstPoints[i], 3, Scalar(255,0,0), 2);

        imshow(windowName, CheckPoints);
        waitKey(0);

        Size outputSize(1280,960);

        { // Position the square in the output image

            int nx = (outputSize.width / 2) - (width/2);

            int dx = nx - LL.x;
            int dy = LL.y;

            for(int i = 0; i < 4; i++)
            {
                dstPoints[i].x += dx;
                dstPoints[i].y += outputSize.height - dy;
            }

        }

        map_matrix = getPerspectiveTransform(srcPoints, dstPoints);

        cout << "Done!" << endl;

        Mat warped;
        warpPerspective(frame, warped, map_matrix, outputSize);

        resize(warped, warped, Size(640,480));

        imshow(windowName, warped);

        metersPerPixel = 1.0 / (((double)width/outputSize.width)*640);

        cout << metersPerPixel << " m/pixel." << endl;

        cout << "Previewing transformed image." << endl;

        waitKey(0);

        cout << "Try again? [y/n]:";
        cout.flush();

        cin >> response;
    } while(response == 'y' || response == 'Y');


    cout << "Save transform matrix? [y/n]:";
    cout.flush();
    cin >> response;

    if(response == 'y' || response == 'Y')
    {
        //imwrite("/home/robojackets/igvc/2013/software/calibration/PTCalibMat.png", map_matrix);

        FileStorage file("/home/robojackets/igvc/2013/software/calibration/PTCalibMat.yml", FileStorage::WRITE);
        file << "mapMatrix" << map_matrix;
        file << "mPerPixel" << metersPerPixel;
        file.release();
    }

    return 0;
}

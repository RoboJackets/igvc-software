#ifndef TESTLINEDETECTION_HPP
#define TESTLINEDETECTION_HPP

#include <QtTest>
#include <intelligence/linedetection/linedetector.h>
#include <iostream>

class TestLineDetection: public QObject
{
    Q_OBJECT
public:
    TestLineDetection() { }

private:
    cv::Mat src, lines;
    bool responded;
    void DisplayImages()
    {
        //Show transformed image
        cv::imshow("Filter Practice", lines);
        //Show original image in a different window
        cv::imshow("Original", src);
    }

    void onResults(ImageData img)
    {
        responded = true;
        lines = img.mat();
        DisplayImages();
    }


//    LISTENER(TestLineDetection, onResults, ImageData)

//private Q_SLOTS:
    // TODO fix this test case. LineDetector constructor call needs to be updated when LineDetector is cleaned up
//    void testCase1()
//    {
//        ///NOTE: The directory may not be the same on your computer!
//        char videoFile[] = "../src/intelligence/igvc_cam_data/stills/img_left2.jpg"; //Still
//        //char videoFile[] = "../src/intelligence/igvc_cam_data/video/CompCourse_left0.mpeg"; //vid

//        cv::VideoCapture cap(videoFile);
//        Event<ImageData> newImageFrameEvent;
//        bool success = cap.read(src);

//        if(!success)
//        {
//            QFAIL("Could not load test video.");
//        }

//        LineDetector ldl(newImageFrameEvent);
//       ldl.onNewLines += &LonResults;

//        while (success){
//            responded = false;
//            newImageFrameEvent(src);
//            // This checks that the LineDetector is actually responding to events
//            QTRY_VERIFY_WITH_TIMEOUT(responded, 1000);
//            cv::waitKey(1);
//            success = cap.read(src);
//        }
//    }
};

#endif // TESTLINEDETECTION_HPP

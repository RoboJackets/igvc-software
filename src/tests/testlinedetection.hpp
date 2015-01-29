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
        if(!lines.empty()) {

            cv::imshow("Filter Practice", lines);
        }
        //Show transformed image
        //Show original image in a different window
        if(!src.empty()) {

            cv::imshow("Original", src);
        }
    }

signals:
    void setImageEvent(ImageData img);

public slots:
    void onResults(ImageData img)
    {
        responded = true;
        lines = img.mat();
        DisplayImages();
    }

//    LISTENER(TestLineDetection, onResults, ImageData)

private Q_SLOTS:
    // TODO fix this test case. LineDetector constructor call needs to be updated when LineDetector is cleaned up
    void testCase1()
    {
        ///NOTE: The directory may not be the same on your computer!
        char videoFile[] = "../test_data/stills/Sun Jun 8 12:08:34 2014img.jpg"; //Still
        //char videoFile[] = "../src/intelligence/igvc_cam_data/video/CompCourse_left0.mpeg"; //vid

        LineDetector ldl;
        connect(&ldl, SIGNAL(onNewLines(ImageData)), this, SLOT(onResults(ImageData)));
        connect(this, SIGNAL(setImageEvent(ImageData)), &ldl, SLOT(onImageEvent(ImageData)));

        src = cv::imread(videoFile);

        setImageEvent(src);
//        ldl.onNewLines += &LonResults;

        bool success = true;

        while (success){
            responded = false;
            // This checks that the LineDetector is actually responding to events
//            QTRY_VERIFY_WITH_TIMEOUT(responded, 1000);
            cv::waitKey(0);
        }
    }
};

#endif // TESTLINEDETECTION_HPP

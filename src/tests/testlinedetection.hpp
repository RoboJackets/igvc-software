#ifndef TESTLINEDETECTION_HPP
#define TESTLINEDETECTION_HPP

#include <QtTest>
#include <intelligence/linedetection/linedetector.h>
#include <iostream>
#include <common/events/Event.hpp>
#ifdef _WIN32
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
#endif
//My event of type <
class TestLineDetection: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testCase1()
    {
        ///NOTE: The directory may not be the same on your computer!
        char videoFile[] = "../src/intelligence/igvc_cam_data/video/CompCourse_left0.mpeg";
        VideoCapture cap(videoFile);
        Mat src;
        Event<ImageData> newImageFrameEvent;
        bool success = cap.read(src);

        if(!success)
        {
            QFAIL("Could not load test video.");
        }

        LineDetector ldl(newImageFrameEvent);

        while (success){
            newImageFrameEvent(src);
            waitKey(1);
            success = cap.read(src);
        }
    }
};

#endif // TESTLINEDETECTION_HPP

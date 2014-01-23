#ifndef TESTLINEDETECTION_HPP
#define TESTLINEDETECTION_HPP

#include <QtTest>
#include <intelligence/linedetection/linedetector.h>
#include <iostream>
#ifdef _WIN32
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
#endif

class TestLineDetection: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testCase1()
    {
        //char videoFile[] = "../igvc_cam_data/stills/img_left2.jpg";
        char videoFile[] = "../src/intelligence/igvc_cam_data/video/CompCourse_left0.mpeg";

         LineDetector ln(videoFile);
         bool success =true;
         while(success){

             ln.applyAlgorithm();
             success = ln.loadImage(videoFile);
         }
    }
};

#endif // TESTLINEDETECTION_HPP

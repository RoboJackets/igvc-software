#ifndef TESTLINEDETECTION_HPP
#define TESTLINEDETECTION_HPP

#include <QtTest>
#include <intelligence/linedetection/linedetector.h>

class TestLineDetection: public QObject
{
    Q_OBJECT

private Q_SLOTS:
    void testCase1()
    {
        //char videoFile[] = "../igvc_cam_data/stills/img_left2.jpg";
        char videoFile[] = "../igvc_cam_data/video/CompCourse_left0.mpeg";
         LineDetector ln(videoFile);
         bool success =true;
         while(success){

             ln.applyAlgorithm();
             success = ln.loadImage(videoFile);
         }
    }
};

#endif // TESTLINEDETECTION_HPP

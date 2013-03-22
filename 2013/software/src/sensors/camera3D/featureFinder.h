#ifndef FEATUREFINDER_H
#define FEATUREFINDER_H

#include "sensors/camera3D/StereoPair.hpp"
#include "sensors/camera3D/StereoSource.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>



class featureFinder
{
    public:
        featureFinder(StereoSource& source);
        void StereoBM(StereoPair& newPair, CvArr* disparity);
        virtual ~featureFinder();
        LISTENER(featureFinder, onNewFrame, StereoPair);
    private:
        void setDefaultBMState(CvStereoBMState& def);
        Event<StereoPair> onNewFrame;
        Event<Mat> onNewDepthMap;
        cv::StereoBM _stereo;
        Mat _disparityMap;
};

#endif // FEATUREFINDER_H

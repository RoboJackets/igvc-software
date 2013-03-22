#include "featureFinder.h"



featureFinder::featureFinder(StereoSource& source) : LonNewFrame(this), _stereo(), _disparityMap()
{
    source.onNewData += &LonNewFrame;
}

void featureFinder::StereoBM(StereoPair& newPair, CvArr* disparity)
{
    _stereo(newPair.LeftImage(), newPair.RightImage(), _disparityMap);
}

/*
void featureFinder::setDefaultBMState(CvStereoBMState& def)
{
    def.preFilterSize=5;
    def.preFilterCap=6;
    def.SADWindowSize = 45;
    def.minDisparity = 0;
    def.numberOfDisparities = 80;
    def.textureThreshold = 0;
    def.uniquenessRatio =2;
    def.speckleWindowSize = 0;
    def.speckleRange = 60;
}
*/

featureFinder::~featureFinder()
{
    //dtor
}

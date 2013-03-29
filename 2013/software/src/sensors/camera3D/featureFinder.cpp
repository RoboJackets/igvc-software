#include "featureFinder.h"



featureFinder::featureFinder(StereoSource& source) : LonNewFrame(this), _stereo(), _disparityMap()
{
    source.onNewData += &LonNewFrame;
    namedWindow("disp", 1);
}

void featureFinder::onNewFrame(StereoPair& newPair)
{
    StereoBM(newPair, &_disparityMap);
}

void featureFinder::StereoBM(StereoPair& newPair, CvArr* disparity)
{
    _useLock.lock();
    Mat lGray, rGray, dis;
    cvtColor( newPair.LeftImage(), lGray, CV_BGR2GRAY );
    cvtColor( newPair.RightImage(), rGray, CV_BGR2GRAY );

    Ptr<CvStereoBMState> def = _stereo.state;
    def->preFilterSize=5;
    def->preFilterCap=6;
    def->SADWindowSize = 45;
    def->minDisparity = 0;
    def->numberOfDisparities = 80;
    def->textureThreshold = 0;
    def->uniquenessRatio =2;
    def->speckleWindowSize = 0;
    def->speckleRange = 60;

    _stereo(rGray, lGray, dis);
    //onNewDepthMap(_disparityMap);
    Mat dispDis;
    convertScaleAbs(dis, dispDis, 5);
    imshow("disp", dispDis);
    //imwrite("this.jpg", dis);
    _useLock.unlock();
    if(waitKey(20) >= 0);
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

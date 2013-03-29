#include "featureFinder.h"
#include "sensors/timing.h"



featureFinder::featureFinder(StereoSource& source) : LonNewFrame(this), _stereo(), _disparityMap()
{
    source.onNewData += &LonNewFrame;
    namedWindow("disp", 1);
    setDefaultBMState(_stereo.state);
}

void featureFinder::onNewFrame(StereoPair& newPair)
{
    StereoBM(newPair, &_disparityMap);
}

void featureFinder::StereoBM(StereoPair& newPair, CvArr* disparity)
{
    double tic,toc;
    tic = seconds_since_IGVCpoch();
    _useLock.lock();
    Mat lGray, rGray, dis;
    cvtColor( newPair.LeftImage(), lGray, CV_BGR2GRAY );
    cvtColor( newPair.RightImage(), rGray, CV_BGR2GRAY );

    _stereo(rGray, lGray, dis);
    toc = seconds_since_IGVCpoch();
    std::cout << "Disparity map took " << toc-tic << " seconds to generate" << std::endl;
    //onNewDepthMap(_disparityMap);
    Mat dispDis;
    double minVal, maxVal;
    minMaxLoc(dis, &minVal, &maxVal);
    dis.convertTo(dispDis, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    imshow("disp", dispDis);
    //imwrite("this.jpg", dis);
    _useLock.unlock();
    tic = seconds_since_IGVCpoch();
    if(waitKey(20) >= 0);

    std::cout << "Disparity map took " << tic-toc << " seconds to display" << std::endl;
}


void featureFinder::setDefaultBMState(Ptr<CvStereoBMState>& state)
{
    state->preFilterSize=5;
    state->preFilterCap=6;
    state->SADWindowSize = 45;
    state->minDisparity = 0;
    state->numberOfDisparities = 80;
    state->textureThreshold = 0;
    state->uniquenessRatio =2;
    state->speckleWindowSize = 0;
    state->speckleRange = 60;
}

featureFinder::~featureFinder()
{
    //dtor
}

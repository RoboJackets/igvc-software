#include "transformer.h"
#include <stdio.h>


using namespace std;

transformer::transformer(Event<ImageData> &evtSrc)
    :LonImageEvent(this)
{
    evtSrc += &LonImageEvent;
    //TODO here put in the coordinates of pc1-pc2
    //As measured in real life
    /// (x1, x2, x3, y1, y2, y3, 1 1 1)

    pcam = (cv::Mat_<float>(2,2) << 427, 511, 642, 642);
    cout<< pcam << endl;

//    427, 642
//    515, 642
//    512, 589
//    432, 588
    //TODO here put in the coordinates of p1-p2
    //which you will get from gimp.
    p = (cv::Mat_<float>(2,2) << 427, 515, 642, 642);
    cv::Mat pcamInv = pcam.inv(cv::DECOMP_LU);
    transformMat = p*pcamInv;
    transformMat = abs(transformMat);
    cout<<transformMat<<endl;
}

void transformer::onImageEvent(ImageData imgd){
    src = imgd.mat();

    cv::Mat dim = (cv::Mat_<float>(2,1)<< src.cols, src.rows);

    dim = transformMat*dim;


    dst = cv::Mat( (int) dim.at<float>(1,0)+1, (int) dim.at<float>(0,0)+1, CV_8UC3);
    cv::Mat newLoc(2,1, CV_32F);

    for(int r=0; r<src.rows; r++){
        for (int c=0; c<src.cols; c++){
            newLoc.at<float>(0,0) = c;
            newLoc.at<float>(1,0) = r;
            newLoc = transformMat*newLoc;
            dst.at<cv::Vec3b>((int)newLoc.at<float>(1,0),(int) newLoc.at<float>(0,0)) = src.at<cv::Vec3b>(r,c);

        }
    }
    onNewLines(ImageData(dst));

}



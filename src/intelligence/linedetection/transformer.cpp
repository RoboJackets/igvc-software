#include "transformer.h"

transformer::transformer(Event<ImageData> &evtSrc)
    :LonImageEvent(this)
{
    evtSrc += &LonImageEvent;
    //TODO here put in the coordinates of pc1-pc2
    //As measured in real life
    /// (x1, x2, x3, y1, y2, y3, 1 1 1)
    pcam = (Mat_<int>(3,3) << 0, 0, 0, 0, 0, 0, 1, 1, 1);

    //TODO here put in the coordinates of p1-p2
    //which you will get from gimp.
    p = Mat_<int>(3,3) << (0, 0, 0, 0, 0, 0, 1, 1, 1);
    pcamInv = pcam.inv();
    transformMat = p*pcamInv;
    addition = transformMat.col(2);
    addition = addition.rowRange(0, 1);
    transformMat = transformMat.rowRange(0,1);
    transformMat = transformMat.colRange(0,1);

}

void transformer::onImageEvent(ImageData imgd){
    src = imgd.mat();
    cv::Vec2b dim = cv::Vec2b(src.rows, src.cols);
    dim = transformMat*dim + addition;
    dst = cv::Mat(dim[0], dim[1], CV_32S);
    cv::Vec2b newLoc;

    for(int r=0; r<src.rows; r++){
        for (int c=0; c<src.cols; c++){
            newLoc = transformMat*src.at<cv::Vec2b>(r, c) + addition;
            dst.at<cv::Vec2b>(newLoc[0], newLoc[1]) = src.at<cv::Vec2b>(r,c);
        }
    }
    onNewLines(ImageData(dst));

}



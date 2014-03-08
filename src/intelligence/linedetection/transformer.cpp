#include "transformer.h"
#include <stdio.h>


using namespace std;

transformer::transformer(Event<ImageData> &evtSrc)
    :LonImageEvent(this)
{
    evtSrc += &LonImageEvent;
    //TODO here put in the coordinates of pc1-pc2
    //As measured in real life
    int offset = 0; //20
    pcam = (cv::Mat_<float>(4,2) << offset-12,72, offset, 72, offset, 60,offset -12, 60);
    //pcam = pcam*3+450;
    //cout<< pcam << endl;

    //    430, 642
    //    513, 642
    //    513, 588
    //    430, 588

//    427, 642
//    515, 642
//    512, 589
//    432, 588

    p = (cv::Mat_<float>(4,2) << 427, 642, 515, 642, 512, 589, 432, 588);
\
    transformMat = cv::getPerspectiveTransform(p, pcam);//cv::findHomography(cv::Mat(p), cv::Mat(pcam));
    cout<<transformMat<<endl;
}

void transformer::onImageEvent(ImageData imgd){
    src = imgd.mat();

    cv::warpPerspective(src, dst, transformMat, dst.size());

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

     // Fill in the cloud data
     cloud->width  = 15;
     cloud->height = 1;
     cloud->points.resize (cloud->width * cloud->height);

     // Generate the data
     for (size_t i = 0; i < cloud->points.size (); ++i)
     {
       cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].z = 1.0;
     }

     // Set a few outliers
     cloud->points[0].z = 2.0;
     cloud->points[3].z = -2.0;
     cloud->points[6].z = 4.0;
    onNewLines(ImageData(dst));

}



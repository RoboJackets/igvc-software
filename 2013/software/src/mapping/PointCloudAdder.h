#ifndef POINTCLOUDADDER_H
#define POINTCLOUDADDER_H

#include "events/Delegate.hpp"
#include <pcl/kdtree/kdtree_flann.h>


class PointCloudAdder
{
  public:
    PointCloudAdder(StereoSource) {}
    LISTENER(PointCloudAdder, onNewSource1, Point)
    LISTENER(PointCloudAdder, onNewSource2, Point)
    onNewSourece1
    onNewSource2()


    virtual ~PointCloudAdder() {}
  protected:
  private:
    pcl::PointCloud<pcl::PointXYZ> _cloud1
    pcl::PointCloud<pcl::PointXYZ> _cloud2
    bool _cloud1Received;
    bool _cloud2Received;
};

#endif // POINTCLOUDADDER_H

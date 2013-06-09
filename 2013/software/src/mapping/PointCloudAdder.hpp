#ifndef POINTCLOUDADDER_H
#define POINTCLOUDADDER_H

#include "events/Delegate.hpp"
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>


typedef PC pcl::PointCloud<pcl::PointXYZ>;

class PointCloudAdder
{
  public:
    PointCloudAdder(Event< PC>* event1, Event<PC>* event2):
    _cloud1Received(false), _cloud2Received(false)
    {
      (*event1)->&LonNewSource1;
      (*event2)->&LonNewSource2;
    }

    LISTENER(PointCloudAdder, onNewSource1, PC);
    LISTENER(PointCloudAdder, onNewSource2, PC);
    Event<PC> onBothData;
    virtual ~PointCloudAdder() {}

  private:
      void onNewSourece1(PC newCloud)
      {
        if (_cloud2Received)
        {
          return _cloud1 += _cloud2;
          _cloud2Received = false;
        }
        else
        {
          _cloud1Received1 = true;
        }
      }

      void onNewSource2(PC newCloud)
      {
        if (_cloud1Received)
        {
          return _cloud1 += _cloud2;
          _cloud1Received = false;
        }
        else
        {
          _cloud1Received2= true;
        }
      }


    pcl::PointCloud<pcl::PointXYZ> _cloud1
    pcl::PointCloud<pcl::PointXYZ> _cloud2
    bool _cloud1Received;
    bool _cloud2Received;
};

#endif // POINTCLOUDADDER_H

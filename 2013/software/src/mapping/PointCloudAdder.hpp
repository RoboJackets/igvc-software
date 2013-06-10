#ifndef POINTCLOUDADDER_H
#define POINTCLOUDADDER_H

#include "events/Delegate.hpp"
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <events/Event.hpp>


typedef pcl::PointCloud<pcl::PointXYZ> PC;

class PointCloudAdder
{
  public:
    PointCloudAdder(Event<PC>* event1, Event<PC>* event2)
        : _cloud1Received(false),
          _cloud2Received(false),
          LonNewSource1(this),
          LonNewSource2(this)
    {
      (*event1) += &LonNewSource1;
      (*event2) += &LonNewSource2;
    }

    LISTENER(PointCloudAdder, onNewSource1, PC);
    LISTENER(PointCloudAdder, onNewSource2, PC);
    Event<PC> onBothData;
    virtual ~PointCloudAdder() {}

  private:
      void onNewSource1(PC newCloud)
      {
        if (_cloud2Received)
        {
          _cloud1 += _cloud2;
          onBothData(_cloud1);
          _cloud2Received = false;
        }
        else
        {
          _cloud1Received = true;
        }
      }

      void onNewSource2(PC newCloud)
      {
        if (_cloud1Received)
        {
          _cloud1 += _cloud2;
          onBothData(_cloud1);
          _cloud1Received = false;
        }
        else
        {
          _cloud2Received= true;
        }
      }


    pcl::PointCloud<pcl::PointXYZ> _cloud1;
    pcl::PointCloud<pcl::PointXYZ> _cloud2;
    bool _cloud1Received;
    bool _cloud2Received;
};

#endif // POINTCLOUDADDER_H

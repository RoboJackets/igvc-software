#ifndef LIDARTOPOINTCLOUDCONVERTER_HPP_INCLUDED
#define LIDARTOPOINTCLOUDCONVERTER_HPP_INCLUDED

#include <sensors/lidar/Lidar.h>
#include <pcl/common/common.h>
#include <events/Event.hpp>

using namespace IGVC::Sensors;

class LidarToPointCloudConverter
{
public:
    LidarToPointCloudConverter(IGVC::Sensors::Lidar* lidar)
        : LOnNewLidar(this)
    {
        if(lidar)
            lidar->onNewData += &LOnNewLidar;
    }

    Event<pcl::PointCloud< pcl::PointXYZ> > OnNewData;

private:
    void OnNewLidar(LidarState data)
    {
        _cloud.points.clear();
        LidarPoint* points = data.points;
        for(int i = 0; i < 1024; i++)
        {
            LidarPoint p = points[i];
            if(p.valid)
            {
                _cloud.points.push_back(pcl::PointXYZ(cos(p.angle)*p.distance, sin(p.angle)*p.distance, 0));
            }
        }
        OnNewData(_cloud);
    }
    LISTENER(LidarToPointCloudConverter, OnNewLidar, LidarState);

    pcl::PointCloud< pcl::PointXYZ > _cloud;
};

#endif // LIDARTOPOINTCLOUDCONVERTER_HPP_INCLUDED

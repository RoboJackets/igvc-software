#include "mapping.h"
#include "hardware/sensors/lidar/Lidar.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <cmath>
#include "intelligence/posetracking/RobotPosition.h"
#include "common/utils/GPSUtils.h"

Mapping::Mapping(IGVC::Sensors::Lidar *lidar, RobotPosition robot):LOnLidarData(this)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    _cloud->height = 1;
    _cloud->width = 1024;
    _cloud->is_dense = false;
    _cloud->points.resize(_cloud->width*_cloud->height);

    if(lidar)
    {
        _lidar = lidar;
        _lidar->onNewData += &LOnLidarData;
    }

    startLat = robot.currentLat();
    startLon = robot.currentLong();

    cloud = _cloud;
}

Mapping::~Mapping()
{
    if(_lidar)
        _lidar->onNewData -= &LOnLidarData;
}

void Mapping::OnLidarData(IGVC::Sensors::LidarState state)
{
    double d = GPSUtils::coordsToMeter(startLat, startLon, robot.currentLat(), robot.currentLong());

    double heading = robot.currentHeading();
    double xDist;
    double yDist;

    if(heading>=0.0 && heading<=90.0)
    {
        xDist = cos(90.0-heading)*d;
        yDist  = sin(90.0-heading)*d;
    }
    else if(heading>90.0 && heading<180.0)
    {
        xDist = cos(heading-90.0)*d;
        yDist = sin(heading-90.0)*d;
    }
    else if(heading>=180.0 && heading<=270.0)
    {
        xDist = cos(270.0-heading)*d;
        yDist = sin(270.0-heading)*d;
    }
    else
    {
        xDist = cos(heading-270.0)*d;
        yDist = sin(heading-270.0)*d;
    }


    for (int i=0;i<1024;i++)
    {
        if(state.points[i].valid)
        {
            cloud->points[i].x = cos(state.points[i].angle)*state.points[i].distance + xDist;
            cloud->points[i].y = sin(state.points[i].angle)*state.points[i].distance + yDist;
            cloud->points[i].z = 0;
        }
    }
    onNewMap(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Mapping::getCloud()
{
    return cloud;
}

/*
void Mapping::saveCloud(std::string path)
{
     pcl::io::savePCDFileASCII(path,cloud);
}


bool Mapping::readCloud(std::string path)
{
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud == -1)
    {
        PCL_ERROR("Couldnt read the cloud\n");
        return(false);
    }
    return(true);
}
*/

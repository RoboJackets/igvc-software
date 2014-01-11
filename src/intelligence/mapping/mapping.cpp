#include "mapping.h"
#include "hardware/sensors/lidar/Lidar.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include "intelligence/posetracking/RobotPosition.h"

Mapping::Mapping(IGVC::Sensors::Lidar *lidar):LOnLidarData(this)
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
    /*
     if(gps)
     {
         startLat = RobotPosition::currentLat();
         startLong = RobotPosition::currentLong();
     }

    float R = 6378.137; // Radius of earth in KM
    var dLat = (lat2 - lat1) * Math.PI / 180;
    var dLon = (lon2 - lon1) * Math.PI / 180;
    var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon/2) * Math.sin(dLon/2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    var d = R * c;
    return d * 1000; // meters
*/
    cloud = _cloud;
}

Mapping::~Mapping()
{
    if(_lidar)
        _lidar->onNewData -= &LOnLidarData;
}

void Mapping::OnLidarData(IGVC::Sensors::LidarState state)
{
    /*
    float lat2 = RobotPosition::currentLat();
    float lon2 = RobotPosition::currentLong();
    float dLat = (lat2 - startLat) * Math.PI / 180;
    float dLon = (lon2 - startLon) * Math.PI / 180;
    float a = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.cos(startLat * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * Math.sin(dLon/2) * Math.sin(dLon/2);
    float c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    float d = R * c * 1000; meters

    float heading = RobotPosition::currentHeading();
    float xDist;
    float yDist;

    if(heading>=0.0 && heading<=90)
    {
        xDist = Math.cos(90.0-heading)*d;
        yDist  = Math.sin(90.0-heading)*d;
    }
    else if(heading>90 && heading<180)
    {
        xDist = Math.cos(heading-90.0)*d;
        yDist = Math.sin(heading-90.0)*d;
    }
    else if(heading>=180 && heading<=270)
    {
        xDist = Math.cos(270.0-heading)*d;
        yDist = Math.sin(270.0-heading)*d;
    }
    else
    {
        xDist = Math.cos(heading-270.0)*d;
        yDist = Math.sin(heading-270.0)*d;
    }

    add xDist and yDist to x and y points
    */

    for (int i=0;i<1024;i++)
    {
        if(state.points[i].valid)
        {
            cloud->points[i].x = cos(state.points[i].angle)*state.points[i].distance;
            cloud->points[i].y = sin(state.points[i].angle)*state.points[i].distance;
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

#include "mapping.h"
#include "hardware/sensors/lidar/Lidar.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

Mapping::Mapping(Lidar *lidar):LOnLidarData(this)
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
    cloud = _cloud;
}

Mapping::~Mapping()
{
    if(_lidar)
        _lidar->onNewData -= &LOnLidarData;
}

void Mapping::OnLidarData(LidarState state)
{
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

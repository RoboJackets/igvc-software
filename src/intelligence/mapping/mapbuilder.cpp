#include "mapbuilder.h"
#include <math.h>
#include <cmath>
#include <common/utils/AngleUtils.h>
#include <pcl/common/transforms.h>


MapBuilder::MapBuilder(Lidar *lidar, BasicPositionTracker *poseTracker):LOnLidarData(this)
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

    this->poseTracker = poseTracker;

    cloud = _cloud;
}

MapBuilder::~MapBuilder()
{
    if(_lidar)
        _lidar->onNewData -= &LOnLidarData;
    poseTracker = nullptr;
}

void MapBuilder::OnLidarData(LidarState state)
{
    RobotPosition pose = poseTracker->GetPosition();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < 1024; i++)
    {
        if(state.points[i].valid)
        {
            pcl::PointXYZ p;
            p.x = cos(state.points[i].angle)*state.points[i].distance;
            p.y = -sin(state.points[i].angle)*state.points[i].distance;
            p.z = 0;
            cloud_lidar->points.push_back(p);
        }
    }

    //Transform current cloud to match robot's position (determined via odometry)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    double mx = -pose.X;
    double my =  pose.Y;
    double mt = AngleUtils::degToRads(pose.Heading);
    Eigen::Vector3f translation(mx, my, 0);
    Eigen::Quaternionf rotation(Eigen::AngleAxisf(mt, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud_lidar, *cloud_transformed, translation, rotation);

    (*cloud) += (*cloud_transformed);

    onNewMap(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapBuilder::getCloud()
{
    return cloud;
}

void MapBuilder::Clear()
{
    cloud->clear();
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

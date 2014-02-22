#include "mapbuilder.h"
#include <math.h>
#include <cmath>
#include <common/utils/AngleUtils.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <common/config/configmanager.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


MapBuilder::MapBuilder(Lidar *lidar, BasicPositionTracker *poseTracker)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    _cloud->height = 1;
    _cloud->width = 1024;
    _cloud->is_dense = false;
    _cloud->points.resize(_cloud->width*_cloud->height);
    firstFrame = true;

    if(lidar)
    {
        _lidar = lidar;
        connect(_lidar, SIGNAL(onNewData(LidarState)), this, SLOT(onLidarData(LidarState)));
    }

    this->poseTracker = poseTracker;

    cloud = _cloud;
}

MapBuilder::~MapBuilder()
{
    if(_lidar)
        disconnect(_lidar, SIGNAL(onNewData(LidarState)), this, SLOT(onLidarData(LidarState)));
    poseTracker = nullptr;
}

void MapBuilder::onLidarData(LidarState state)
{
    RobotPosition pose = poseTracker->GetPosition();
    double lidarOffset_y = ConfigManager::Instance().getValue("MapBuilder", "lidarOffset_y", 0);
    double lidarOffset_x = ConfigManager::Instance().getValue("MapBuilder", "lidarOffset_x", 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < 1024; i++)
    {
        if(state.points[i].valid)
        {
            pcl::PointXYZ p;
            p.x = cos(state.points[i].angle)*state.points[i].distance + lidarOffset_x;
            p.y = -sin(state.points[i].angle)*state.points[i].distance - lidarOffset_y;
            p.z = 0;
            cloud_lidar->points.push_back(p);
        }
    }

    //Transform current cloud to match robot's position (determined via odometry)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    double mx =  pose.X;
    double my =  -pose.Y;
    double mt = AngleUtils::degToRads(pose.Heading);
    Eigen::Vector3f translation(mx, my, 0);
    Eigen::Quaternionf rotation(Eigen::AngleAxisf(mt, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud_lidar, *cloud_transformed, translation, rotation);

    // ICP to refine cloud alignment
    if(!firstFrame)
    {
        //Use ICP to fix errors in transformation
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(2);
        icp.setMaximumIterations(30);
        icp.setInputCloud(cloud_transformed);
        icp.setInputTarget(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*cloud_aligned);

        if(icp.hasConverged())
        {
            //Add aligned cloud points to map cloud
            (*cloud) += *cloud_aligned;
        }
        else
        {
            //Ignore alignment atempt
            std::cout << "No convergence." << std::endl;
            (*cloud) += *cloud_transformed;
        }
    }
    else
    {
        //Add first cloud's points to map cloud
        (*cloud) += *cloud_transformed;
    }

    //Run map cloud through a VoxelGrid to remove any duplicate points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedDuplicates (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> duplicateRemover;

    duplicateRemover.setInputCloud(cloud);
    duplicateRemover.setLeafSize (0.05f, 0.05f, 0.05f);
    duplicateRemover.filter(*cloud_removedDuplicates);

    cloud.swap(cloud_removedDuplicates);

    onNewMap(cloud);
    firstFrame = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapBuilder::getCloud()
{
    return cloud;
}

void MapBuilder::Clear()
{
    cloud->clear();
}

void MapBuilder::ChangeLidar(Lidar *device)
{
    if(_lidar != nullptr)
    {
        disconnect(_lidar, SIGNAL(onNewData(LidarState)), this, SLOT(onLidarData(lidarState)));
    }
    _lidar = device;
    if(_lidar != nullptr)
    {
        connect(_lidar, SIGNAL(onNewData(LidarState)), this, SLOT(onLidarData(lidarState)));
    }

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

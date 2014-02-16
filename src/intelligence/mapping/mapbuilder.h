#ifndef MAPPING_H
#define MAPPING_H

#include "hardware/sensors/lidar/Lidar.h"
#include "common/events/Event.hpp"
#include <pcl/point_types.h>
#include <pcl/io/file_io.h>
#include <intelligence/posetracking/basicpositiontracker.h>

/*!
 * \brief Maps the course from sensor data.
 * \author Al Chaussee
 */
class MapBuilder
{
public:
    MapBuilder(Lidar *lidar, BasicPositionTracker *poseTracker);
    ~MapBuilder();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
    //void saveCloud(std::string path);
    //bool readCloud(std::string path);
    Event<pcl::PointCloud<pcl::PointXYZ>::Ptr> onNewMap;

    void Clear();

private:

    void OnLidarData(LidarState state);
    LISTENER(MapBuilder, OnLidarData, LidarState)

    Lidar *_lidar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    BasicPositionTracker *poseTracker;
};

#endif // MAPPING_H

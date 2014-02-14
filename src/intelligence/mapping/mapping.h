#ifndef MAPPING_H
#define MAPPING_H

#include "hardware/sensors/lidar/Lidar.h"
#include "common/events/Event.hpp"
#include <pcl/point_types.h>
#include <pcl/io/file_io.h>

/*!
 * \brief Maps the course from sensor data.
 * \author Al Chaussee
 */
class Mapping
{
public:
    Mapping(Lidar *lidar);
    ~Mapping();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
    //void saveCloud(std::string path);
    //bool readCloud(std::string path);
    Event<pcl::PointCloud<pcl::PointXYZ>::Ptr> onNewMap;
private:
    Lidar *_lidar;
    void OnLidarData(LidarState state);
    LISTENER(Mapping, OnLidarData, LidarState)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif // MAPPING_H

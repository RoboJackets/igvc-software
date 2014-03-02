#ifndef MAPPING_H
#define MAPPING_H

#include "hardware/sensors/lidar/Lidar.h"
#include <pcl/point_types.h>
#include <pcl/io/file_io.h>
#include <intelligence/posetracking/basicpositiontracker.h>

/*!
 * \brief Maps the course from sensor data.
 * \author Al Chaussee
 */
class MapBuilder : public QObject
{
    Q_OBJECT
public:
    MapBuilder(Lidar *lidar, BasicPositionTracker *poseTracker);
    ~MapBuilder();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
    //void saveCloud(std::string path);
    //bool readCloud(std::string path);

    void Clear();

    void ChangeLidar(Lidar *device);

signals:
    void onNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr);

private slots:

    void onLidarData(LidarState state);

private:

    Lidar *_lidar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    BasicPositionTracker *poseTracker;
    bool firstFrame;
};

#endif // MAPPING_H

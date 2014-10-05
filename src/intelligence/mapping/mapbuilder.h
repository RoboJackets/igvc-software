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
    MapBuilder(std::shared_ptr<Lidar> lidar, std::shared_ptr<BasicPositionTracker> poseTracker);
    ~MapBuilder();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
    //void saveCloud(std::string path);
    //bool readCloud(std::string path);

    void Clear();

    void ChangeLidar(std::shared_ptr<Lidar> device);

signals:
    void onNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr);

private slots:

    void onLidarData(LidarState state);
    void onCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr frame, pcl::PointXY sensorOffset);

private:

    std::shared_ptr<Lidar> _lidar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::shared_ptr<BasicPositionTracker> poseTracker;
};

#endif // MAPPING_H

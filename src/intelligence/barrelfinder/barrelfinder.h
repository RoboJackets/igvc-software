#ifndef BARRELFINDER_H
#define BARRELFINDER_H

#include <QObject>
#include <common/datastructures/ImageData.hpp>
#include <pcl/common/common.h>

class BarrelFinder : public QObject
{
    Q_OBJECT
public:
    BarrelFinder(QObject *parent = 0);

signals:
    void newCloudFrame(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

public slots:
    void onNewImage(ImageData data);

private:
    pcl::PointCloud<pcl::PointXYZ> cloud;

};

#endif // BARRELFINDER_H

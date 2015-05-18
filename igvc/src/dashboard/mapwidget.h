#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MapWidget : public QWidget
{
    Q_OBJECT

public:
    MapWidget(QWidget *parent);

public slots:
    void setMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr value);

protected:
    void paintEvent(QPaintEvent *) override;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr map;
};

#endif

#ifndef MAPADAPTER_H
#define MAPADAPTER_H

#include <QWidget>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <intelligence/mapping/mapbuilder.h>
#include <intelligence/posetracking/basicpositiontracker.h>

namespace Ui {
class MapAdapter;
}

/*!
 * \brief Widget for displaying Map data.
 * \author Matthew Barulic
 */
class MapAdapter : public QWidget
{
    Q_OBJECT
    
public:
    explicit MapAdapter(MapBuilder *mapper, BasicPositionTracker *posTracker, QWidget *parent = 0);
    ~MapAdapter();

protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent *e);
    
private slots:
    void on_scaleSlider_sliderMoved(int position);

    void on_homeButton_clicked();

    void on_pushButton_clicked();

    void onNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map);

private:
    Ui::MapAdapter *ui;

    // pixels per meter
    double _scale;

    // on-screen displacement of map's origin from widget's center
    QPoint _origin;

    bool _isDragging;
    QPoint _previousMousePosition;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _map;

    void setScale(int scale);

    void paintEvent(QPaintEvent *);

    MapBuilder *_mapper;

    BasicPositionTracker *_posTracker;
};

#endif // MAPADAPTER_H

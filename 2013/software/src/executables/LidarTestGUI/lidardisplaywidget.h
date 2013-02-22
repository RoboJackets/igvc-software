#ifndef LIDARDISPLAYWIDGET_H
#define LIDARDISPLAYWIDGET_H

#include <QWidget>
#include "sensors/lidar/Lidar.h"
#include <boost/thread.hpp>
#include "mapping/extractors/lidarobstacleextractor.h"

using namespace IGVC::Sensors;

class LidarDisplayWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LidarDisplayWidget(QWidget *parent = 0);

    void setScale(double scale);
    double scale();

    void capture();

    void setLidar(Lidar *device);

    enum ViewMode {
        Points,
        Lines
    };

    void setViewMode(ViewMode mode);

    bool event(QEvent * event);

    bool shouldUpdateOnScaling;

protected:
    void paintEvent(QPaintEvent *event);

private:
    void onNewLidarData(LidarState state);

    LISTENER(LidarDisplayWidget, onNewLidarData, LidarState)

    LidarState _lidarData;
    boost::mutex _locker;
    
    Lidar *_lidar;

    ViewMode _vMode;

    double _scale;

    bool _drawing;

    bool _isDragging;

    QPointF _origin;

    LidarObstacleExtractor _lidObstExtractor;
    std::vector<Obstacle*> _obstacles;
};

#endif // LIDARDISPLAYWIDGET_H

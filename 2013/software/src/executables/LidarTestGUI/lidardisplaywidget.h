#ifndef LIDARDISPLAYWIDGET_H
#define LIDARDISPLAYWIDGET_H

#include <QWidget>
#include "sensors/lidar/SimulatedLidar.h"

using namespace IGVC::Sensors;

class LidarDisplayWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LidarDisplayWidget(QWidget *parent = 0);

    void setScale(double scale);
    double scale();

protected:
    void paintEvent(QPaintEvent *event);

private:
    void onNewLidarData(LidarState state);

    LISTENER(LidarDisplayWidget, onNewLidarData, LidarState)

    LidarState _lidarData;
    boost::mutex _locker;
    
    SimulatedLidar _lidar;

    double _scale;

    bool _drawing;
};

#endif // LIDARDISPLAYWIDGET_H

#ifndef LIDARADAPTER_H
#define LIDARADAPTER_H

#include <QWidget>
#include <hardware/sensors/lidar/Lidar.h>

namespace Ui {
class LidarAdapter;
}

class LidarAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit LidarAdapter(IGVC::Sensors::Lidar *lidar, QWidget *parent = 0);
    ~LidarAdapter();

protected:
    void paintEvent(QPaintEvent *);

private slots:
    void on_btn_fit_clicked();
    void on_btn_slider_actionTriggered(int);

private:
    Ui::LidarAdapter *ui;
    bool isFit;
    double fitRange;

    int NUMPTS;

    IGVC::Sensors::Lidar *_lidar;
    IGVC::Sensors::LidarState _data;

    void OnLidarData(IGVC::Sensors::LidarState state);
    LISTENER(LidarAdapter, OnLidarData, IGVC::Sensors::LidarState)
};

#endif // LIDARADAPTER_H

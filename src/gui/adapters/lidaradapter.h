#ifndef LIDARADAPTER_H
#define LIDARADAPTER_H

#include <QWidget>
#include "hardware/sensors/lidar/Lidar.h"

namespace Ui {
class LidarAdapter;
}

class LidarAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit LidarAdapter(QWidget *parent = 0);
    ~LidarAdapter();

protected:
    void paintEvent(QPaintEvent *);

private slots:
    void on_btn_fit_clicked();
    void on_btn_slider_actionTriggered(int action);

private:
    Ui::LidarAdapter *ui;
    double dataPts[1024];
    bool isFit;
    double fitRange;

    int NUMPTS;

    //IGVC::Sensors::LidarState currState;
    //void OnLidarData(IGVC::Sensors::LidarState state);
    //LISTENER(lidaradapter, OnLidarData, IGVC::Sensors::LidarState);
   // bool hasState;
};

#endif // LIDARADAPTER_H

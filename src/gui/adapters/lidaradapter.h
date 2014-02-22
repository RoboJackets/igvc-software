#ifndef LIDARADAPTER_H
#define LIDARADAPTER_H

#include <QWidget>
#include <hardware/sensors/lidar/Lidar.h>

namespace Ui {
class LidarAdapter;
}

/*!
 * \brief Widget for displaying LIDAR data.
 * \author Alexander Huynh
 */
class LidarAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit LidarAdapter(Lidar *lidar, QWidget *parent = 0);
    ~LidarAdapter();

protected:
    void paintEvent(QPaintEvent *);

private slots:
    void on_btn_fit_clicked();
    void on_btn_slider_actionTriggered(int);
    void onLidarData(LidarState state);

private:
    Ui::LidarAdapter *ui;
    bool isFit;
    double fitRange;

    int NUMPTS;

    Lidar *_lidar;
    LidarState _data;
};

#endif // LIDARADAPTER_H

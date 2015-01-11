#ifndef IMUADAPTER_H
#define IMUADAPTER_H

#include <QWidget>
#include <hardware/sensors/IMU/IMU.h>
#include <vector>
#include <memory>

namespace Ui {
class IMUAdapter;
}

/*!
 * \brief Widget for displaying IMU data.
 * \author Matthew Barulic
 */
class IMUAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit IMUAdapter(std::shared_ptr<IMU> imu, QWidget *parent = 0);
    ~IMUAdapter();

private slots:
    void paintEvent(QPaintEvent *);

    void onNewData(IMUData data);

private:
    Ui::IMUAdapter *ui;

    std::shared_ptr<IMU> _imu;

    std::vector<IMUData> _data;

    bool _lock;

    double prevTime;
    double fps;
};

#endif // IMUADAPTER_H

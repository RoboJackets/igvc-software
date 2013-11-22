#ifndef IMUADAPTER_H
#define IMUADAPTER_H

#include <QWidget>
#include <hardware/sensors/IMU/IMU.h>
#include <vector>

namespace Ui {
class IMUAdapter;
}

class IMUAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit IMUAdapter(IMU *imu, QWidget *parent = 0);
    ~IMUAdapter();

private slots:
    void paintEvent(QPaintEvent *);

private:
    Ui::IMUAdapter *ui;

    IMU *_imu;

    std::vector<IMUData> _data;

    bool _lock;

    void OnNewData(IMUData data);
    LISTENER(IMUAdapter, OnNewData, IMUData);
};

#endif // IMUADAPTER_H

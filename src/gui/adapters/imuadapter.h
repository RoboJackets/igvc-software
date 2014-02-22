#ifndef IMUADAPTER_H
#define IMUADAPTER_H

#include <QWidget>
#include <hardware/sensors/IMU/IMU.h>
#include <vector>

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
    explicit IMUAdapter(IMU *imu, QWidget *parent = 0);
    ~IMUAdapter();

private slots:
    void paintEvent(QPaintEvent *);

    void onNewData(IMUData data);

private:
    Ui::IMUAdapter *ui;

    IMU *_imu;

    std::vector<IMUData> _data;

    bool _lock;
};

#endif // IMUADAPTER_H

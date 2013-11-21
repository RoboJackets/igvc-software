#include "imuadapter.h"
#include "ui_imuadapter.h"
#include <QPainter>

IMUAdapter::IMUAdapter(IMU *imu, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::IMUAdapter),
    LOnNewData(this)
{
    ui->setupUi(this);

    if(imu)
    {
        _imu = imu;
        _imu->onNewData += &LOnNewData;
    }
}

IMUAdapter::~IMUAdapter()
{
    _imu->onNewData -= &LOnNewData;
    delete ui;
}

void IMUAdapter::OnNewData(IMUData data)
{
    _data.push_back(data);
    if(_data.size() > 10)
        _data.erase(_data.begin());
    update();
}

void IMUAdapter::paintEvent(QPaintEvent *)
{
    if(_data.size() > 0)
    {
        QPainter painter(this);

        double MAX = M_2_PI + 0.5;
        double dX = this->width() / _data.size();
        double X = 0;

        double Y0 = ui->label->y() / 2.0;

        IMUData prevItem = _data.at(0);
        for(IMUData item : _data)
        {
            painter.setPen(Qt::blue);
            painter.drawLine(X, Y0 - ( prevItem.Roll() / MAX), X + dX, Y0 - ( item.Roll() / MAX ) );
            painter.setPen(Qt::green);
            painter.drawLine(X, Y0 - ( prevItem.Pitch() / MAX), X + dX, Y0 - ( item.Pitch() / MAX ) );
            painter.setPen(Qt::red);
            painter.drawLine(X, Y0 - ( prevItem.Yaw() / MAX), X + dX, Y0 - ( item.Yaw() / MAX ) );

            X += dX;
            prevItem = item;
        }

        IMUData recent = _data.at(_data.size()-1);
        ui->label->setText(tr("Roll: %1\tPitch: %2\tYaw: %3").arg(recent.Roll(),6).arg(recent.Pitch(),6).arg(recent.Yaw(),6));
    }
}

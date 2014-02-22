#include "imuadapter.h"
#include "ui_imuadapter.h"
#include <QPainter>

IMUAdapter::IMUAdapter(IMU *imu, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::IMUAdapter)
{
    ui->setupUi(this);

    if(imu)
    {
        _imu = imu;
        connect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewData(IMUData)));
    }
}

IMUAdapter::~IMUAdapter()
{
    disconnect(_imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewData(IMUData)));
    delete ui;
}

void IMUAdapter::onNewData(IMUData data)
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

        double MAX_RPY = 180.0;
        double MAX_XYZ = 20.0;
        double dX = this->width() / _data.size();
        double X = 0;

        double Y_RPY = ui->labelRoll->y() / 2.0;
        double Y_XYZ =ui->labelX->y() / 2.0 + Y_RPY;

        double H_RPY = ui->labelRoll->y() / 2.0;
        double H_XYZ = ( ui->labelX->y() - ( ui->labelRoll->y() + ui->labelRoll->height() ) ) / 2.0;

        double R_RPY = H_RPY / MAX_RPY;
        double R_XYZ = H_XYZ / MAX_XYZ;

        IMUData prevItem = _data.at(0);
        for(IMUData item : _data)
        {
            painter.setPen(Qt::blue);
            painter.drawLine(X, Y_RPY - ( prevItem.Roll * R_RPY), X + dX, Y_RPY - ( item.Roll * R_RPY) );
            painter.drawLine(X, Y_XYZ - ( prevItem.X * R_XYZ), X + dX, Y_XYZ - ( item.X * R_XYZ) );
            painter.setPen(Qt::green);
            painter.drawLine(X, Y_RPY - ( prevItem.Pitch * R_RPY), X + dX, Y_RPY - ( item.Pitch * R_RPY) );
            painter.drawLine(X, Y_XYZ - ( prevItem.Y * R_XYZ), X + dX, Y_XYZ - ( item.Y * R_XYZ) );
            painter.setPen(Qt::red);
            painter.drawLine(X, Y_RPY - ( prevItem.Yaw * R_RPY), X + dX, Y_RPY - ( item.Yaw * R_RPY ) );
            painter.drawLine(X, Y_XYZ - ( prevItem.Z * R_XYZ), X + dX, Y_XYZ - ( item.Z * R_XYZ ));

            X += dX;
            prevItem = item;
        }

        IMUData recent = _data.at(_data.size()-1);
        ui->labelRoll->setText(tr("Roll: %1").arg(recent.Roll));
        ui->labelPitch->setText(tr("Pitch: %1").arg(recent.Pitch));
        ui->labelYaw->setText(tr("Yaw: %1").arg(recent.Yaw));
        ui->labelX->setText(tr("X: %1").arg(recent.X));
        ui->labelY->setText(tr("Y: %1").arg(recent.Y));
        ui->labelZ->setText(tr("Z: %1").arg(recent.Z));
    }
}

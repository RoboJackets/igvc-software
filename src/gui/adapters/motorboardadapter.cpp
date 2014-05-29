#include "motorboardadapter.h"
#include "ui_motorboardadapter.h"
#include <QPainter>
#include <iostream>

MotorBoardAdapter::MotorBoardAdapter(std::shared_ptr<MotorDriver> driver, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MotorBoardAdapter)
{
    ui->setupUi(this);
    _driver = driver;
    connect(_driver.get(), SIGNAL(newCurrentVelocities(double,double)), this, SLOT(onNewCurrentVelocities(double,double)));
}

MotorBoardAdapter::~MotorBoardAdapter()
{
    disconnect(_driver.get(), SIGNAL(newCurrentVelocities(double,double)), this, SLOT(onNewCurrentVelocities(double,double)));
    delete ui;
}

void MotorBoardAdapter::onNewCurrentVelocities(double left, double right)
{
    _data.push_back(VelocityPairs(VelocityPair(_driver->getLeftSetVelocity(), _driver->getRightSetVelocity()), VelocityPair(left, right)));
    if(_data.size() > 10)
        _data.erase(_data.begin());
    update();
}

void MotorBoardAdapter::paintEvent(QPaintEvent *)
{
    if(_data.size() > 0)
    {
        QPainter painter(this);

        double MAX = 6.0;
        double dX = this->width() / _data.size();
        double X = 0;

        double Y_MID = ui->label->y() / 2;

        double H = ui->label->y() / 2;

        double R = H / MAX;

        painter.setPen(Qt::black);
        painter.drawLine(0, Y_MID, this->width(), Y_MID);

        VelocityPairs prevItem = _data.at(0);
        for(VelocityPairs item : _data)
        {
            painter.setPen(Qt::blue);
            painter.drawLine(X, Y_MID - ( prevItem.first.first * R), X + dX, Y_MID - ( item.first.first * R));

            painter.setPen(Qt::red);
            painter.drawLine(X, Y_MID - ( prevItem.first.second * R), X + dX, Y_MID - (item.first.second * R));

            painter.setPen(Qt::green);
            painter.drawLine(X, Y_MID - ( prevItem.second.first * R), X + dX, Y_MID - ( item.second.first * R));

            painter.setPen(Qt::magenta);
            painter.drawLine(X, Y_MID - ( prevItem.second.second * R), X + dX, Y_MID - (item.second.second * R));

            X += dX;
            prevItem = item;
        }
    }
}

void MotorBoardAdapter::on_lineEdit_leftVel_returnPressed()
{
    bool success;
    double vel = ui->lineEdit_leftVel->text().toDouble(&success);
    if(success)
    {
        _driver->setLeftVelocity(vel);
        ui->lineEdit_leftVel->setStyleSheet(tr(""));
    }
    else
    {
        ui->lineEdit_leftVel->setStyleSheet(tr("background:red"));
    }
}

void MotorBoardAdapter::on_lineEdit_rightVel_returnPressed()
{
    bool success;
    double vel = ui->lineEdit_rightVel->text().toDouble(&success);
    if(success)
    {
        _driver->setRightVelocity(vel);
        ui->lineEdit_rightVel->setStyleSheet(tr(""));
    }
    else
    {
        ui->lineEdit_rightVel->setStyleSheet(tr("background:red"));
    }
}

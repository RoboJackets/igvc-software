#include "motorboardadapter.h"
#include "ui_motorboardadapter.h"

MotorBoardAdapter::MotorBoardAdapter(MotorDriver *driver, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MotorBoardAdapter)
{
    ui->setupUi(this);
    _driver = driver;
    connect(_driver, SIGNAL(newCurrentVelocities(double,double)), this, SLOT(onNewCurrentVelocities(double,double)));
}

MotorBoardAdapter::~MotorBoardAdapter()
{
    delete ui;
}

void MotorBoardAdapter::onNewCurrentVelocities(double left, double right)
{
    ui->label_leftVel->setText(tr("%1").arg(left));
    ui->label_rightVel->setText(tr("%1").arg(right));
    ui->label_leftCmd->setText(tr("%1").arg(_driver->getLeftSetVelocity()));
    ui->label_rightCmd->setText(tr("%1").arg(_driver->getRightSetVelocity()));
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

#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDateTime>


CameraAdapter::CameraAdapter(QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    connect(ui->saveLeft,SIGNAL(released()),SLOT(on_saveLeft_clicked()));

    prevTimeRight = 0;
    prevTimeLeft = 0;
    fpsRight = 0;
    fpsLeft = 0;
}

CameraAdapter::~CameraAdapter()
{
    delete ui;
}

void CameraAdapter::newLeftCamImg(ImageData data)
{
    if(isVisible())
    {
        _mutex.lock();
        leftData = data;
        leftImage = CVMat2QImage(data.mat());
        _mutex.unlock();
        fpsLeft = 1.0/(data.time() - prevTimeLeft);
        prevTimeLeft = data.time();
        update();
    }
}

void CameraAdapter::newRightCamImg(ImageData data)
{
    if(isVisible())
    {
        _mutex.lock();
        rightData = data;
        rightImage = CVMat2QImage(data.mat());
        _mutex.unlock();
        fpsRight = 1.0/(data.time() - prevTimeRight);
        prevTimeRight = data.time();
        update();
    }
}

QImage CameraAdapter::CVMat2QImage(cv::Mat img)
{
    cv::Mat temp(img.cols, img.rows, img.type());

    cv::cvtColor(img, temp, CV_BGR2RGB);

    QImage dest((uchar*)temp.data,temp.cols,temp.rows, temp.step, QImage::Format_RGB888);
    QImage dest2(dest);
    dest2.detach();
    return dest2;
}

void CameraAdapter::on_saveLeft_clicked()
{
    _mutex.lock();
    cv::imwrite((QDateTime::currentDateTime().toString()+"_left.jpg").toStdString(),  leftData.mat());
    _mutex.unlock();
}

void CameraAdapter::on_saveRight_clicked()
{
    _mutex.lock();
    cv::imwrite((QDateTime::currentDateTime().toString()+"_right.jpg").toStdString(),  rightData.mat());
    _mutex.unlock();
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    ui->leftFeedLabel->setPixmap(QPixmap::fromImage(leftImage));
    ui->rightFeedLabel->setPixmap(QPixmap::fromImage(rightImage));
    QString str;
    str.sprintf("FPS %f", fpsRight);
    ui->rightFpsLabel->setText(str);
    str.sprintf("FPS %f", fpsLeft);
    ui->leftFpsLabel->setText(str);

    _mutex.unlock();
    QWidget::paintEvent(e);
}

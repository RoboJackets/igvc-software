#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDateTime>

CameraAdapter::CameraAdapter(std::shared_ptr<StereoSource> source, std::shared_ptr<LineDetector> source2, QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter)
{
    ui->setupUi(this);

    if(source.get() != nullptr)
    {
        _stereoSource = source;
        connect(_stereoSource.get(), SIGNAL(onNewData(StereoImageData)), this, SLOT(onCameraData(StereoImageData)));
    }



    if (source2.get() != nullptr)
    {
        cout<<"I have a source!"<<endl;
        _lineDetector = source2;
        connect(_lineDetector.get(), SIGNAL(onNewLinesMat(cv::Mat)), this, SLOT(onLineImage(cv::Mat)));
    }

    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
    gotData = false;
    connect(ui->saveLeft,SIGNAL(released()),SLOT(on_saveLeft_clicked()));
}

CameraAdapter::~CameraAdapter()
{
    if(_stereoSource.get() != nullptr)
        disconnect(_stereoSource.get(), SIGNAL(onNewData(StereoImageData)), this, SLOT(onCameraData(StereoImageData)));
    delete ui;
}

void CameraAdapter::onCameraData(StereoImageData data)
{
    if(isVisible())
    {
        _mutex.lock();
        _data = data;
        gotData = true;
        _mutex.unlock();
        update();
    }
}

void CameraAdapter::onLineImage(cv::Mat img){
    _mutex.lock();
    rightImage = CVMat2QImage(img);
    _mutex.unlock();
   // update();
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
    //QDir().mkdir("../Images/");
    //leftImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_left.jpg");
    _mutex.lock();
    std::cout << _data.leftMat().rows << std::endl;
    std::cout << cv::imwrite("img.jpg", _data.leftMat()) << std::endl;
    _mutex.unlock();
}

void CameraAdapter::on_saveRight_clicked()
{
    QDir().mkdir("../Images/");
    rightImage.save("../Images/"+QDateTime::currentDateTime().toString()+"_right.jpg");
}

void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    if(gotData){
        leftImage = CVMat2QImage(_data.left().mat());
        //rightImage = CVMat2QImage(_data.right().mat());
        ui->leftFeedLabel->setPixmap(QPixmap::fromImage(leftImage));
        ui->rightFeedLabel->setPixmap(QPixmap::fromImage(rightImage));
    }

    _mutex.unlock();
    QWidget::paintEvent(e);
}

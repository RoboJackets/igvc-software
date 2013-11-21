#include "cameraadapter.h"
#include "ui_cameraadapter.h"
#include "cameraadaptertester.h"
#include <QGraphicsItem>
#include <QPixmap>
#include <iostream>
#include <QGridLayout>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "common/events/Event.hpp"

CameraAdapter::CameraAdapter(CameraAdapterTester *camera, QWidget *parent) :
     QWidget(parent),
     ui(new Ui::CameraAdapter),
     LOnCameraData(this)
{
    ui->setupUi(this);

    if(camera)
    {
        _camera = camera;
        _camera->onNewData += &LOnCameraData;
    }
    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
    gotData = false;
}

CameraAdapter::~CameraAdapter()
{
    if(_camera)
        _camera->onNewData -= &LOnCameraData;
    delete ui;
}

void CameraAdapter::OnCameraData(CameraData data)
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


QImage CameraAdapter::CVMat2QImage(cv::Mat img)
{
    cv::Mat temp(img.cols, img.rows, img.type());
    //cv::cvtColor(img, temp, CV_BayerGR2RGB);

    QImage dest((uchar*)img.data,img.cols,img.rows, img.step, QImage::Format_RGB888);
    QImage dest2(dest);
    dest2.detach();
    return dest2;
}


void CameraAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();

    if(gotData){

       ui->leftFeedLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.leftFeed)));
       ui->rightFeedLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.rightFeed)));
        ui->depthMapLabel->setPixmap(QPixmap::fromImage(CVMat2QImage(_data.depthMap)));
        ui->pointCloudLabel->setPixmap(QPixmap::fromImage( CVMat2QImage(_data.pointCloud)));

    }


    _mutex.unlock();
    QWidget::paintEvent(e);
}

/*void CameraAdapter::resizeEvent(QResizeEvent *e)
{
    QSize size = currentLeftFrame.size();
    cout << size.width() << ", " << size.height() << "\n";
    size.scale(this->size().boundedTo(size), Qt::KeepAspectRatio);
    currentLeftFrame = currentLeftFrame.scaled(size, Qt::KeepAspectRatio);

    this->resize(size);


    ui->tabWidget->resize(this->sizeHint());


    QWidget::resizeEvent(e);

}*/

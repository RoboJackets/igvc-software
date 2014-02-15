#include "lidaradapter.h"
#include "ui_lidaradapter.h"
#include <QPainter>
#include <QCheckBox>
#include <cmath>
#include <QDebug>


LidarAdapter::LidarAdapter(Lidar *lidar, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarAdapter),
    LOnLidarData(this)
{
    ui->setupUi(this);
    NUMPTS = 1024;

    isFit = false;

    _lidar = lidar;
    if(_lidar != nullptr)
        _lidar->onNewData += &LOnLidarData;
}

LidarAdapter::~LidarAdapter()
{
    if(_lidar != nullptr)
        _lidar->onNewData -= &LOnLidarData;
    delete ui;
}

void LidarAdapter::paintEvent(QPaintEvent *)
{
    int range;
    if (!isFit)
        range = ui->btn_slider->value() + 1;
    else
        range = fitRange;
        // 0 to use longest distance in given data for ellipse bound
        // otherwise, ellipse bound is currently selected RANGES value (determined by btn_slider)

    QPainter p(this);

    /*
     *  first constrain drawing region to largest possible circle
     */

    double wid = size().width();    // speed up data access
    double hei = size().height() - 1;

    double centerX = wid * .5;      // center of widget
    double centerY = hei * .5;

    // constrain graphics to square size
    double minDimension = wid;      // smallest of width and height
    double minOffset = hei - wid;   // difference in sizes
    bool minIsWidth = true;         // TRUE if width is smallest; false if height is smallest
    if (hei < minDimension)
    {
        minDimension = hei;
        minOffset = wid - hei;
        minIsWidth = false;
    }

    // draw limit ellipse
    p.setPen(QPen(QColor(0, 0, 0, 128)));
    p.drawEllipse((minIsWidth ? 0 : minOffset * .5),
                   (minIsWidth ? minOffset * .5 : 0),
                   wid - (minIsWidth ? 0 : minOffset),
                   hei - (minIsWidth ? minOffset: 0));

    ui->tf_range->setText(QString("Range: %1").arg(range / 1000.0)); // mm to m

    /*
     *  now draw lidar rays using relative distances;
     *  furthest distance is largest line and all other lines use a % of the longest
     */

    p.setPen(QPen(QColor(0, 0, 0, 32)));
    double mag;         // % of longest line

    for(LidarPoint point : _data.points)
    {
        if(point.valid)
        {
            mag = point.distance / ( range / 1000.0 );
            if(mag > 1)
                mag = 1;
            p.drawLine(centerX, centerY,
                       centerX + cos(point.angle) * mag * minDimension * .5,
                       centerY - sin(point.angle) * mag * minDimension * .5);
        }
    }
}

void LidarAdapter::OnLidarData(LidarState state)
{
    _data = state;
    update();
}

// TODO : Maybe add input textfield for range

void LidarAdapter::on_btn_fit_clicked()
{
    double maxSize = 0;
    for(LidarPoint point : _data.points)
        if(point.valid)
            if(point.distance > maxSize)
                maxSize = point.distance;

    fitRange = maxSize * 1000.0; // m to mm
    isFit = true;
    ui->btn_slider->setSliderPosition(fitRange);
    update();
}

void LidarAdapter::on_btn_slider_actionTriggered(int)
{
    isFit = false;
    update();
}

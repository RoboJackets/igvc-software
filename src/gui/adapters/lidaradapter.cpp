#include "lidaradapter.h"
#include "ui_lidaradapter.h"
#include <QPainter>
#include <QCheckBox>
#include <cmath>
#include <QDebug>


LidarAdapter::LidarAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarAdapter)
{
    ui->setupUi(this);
    NUMPTS = 1024;

    for (int i = 0; i < NUMPTS; i++)
        dataPts[i] = rand() % 4000 + 1;

    isFit = false;
    //hasState = false;
}

LidarAdapter::~LidarAdapter()
{
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

    //if (!hasState)
    //    return;


    // TODO : make this parse a LidarState object.

    ui->tf_range->setText(QString("Range: %1").arg(round(range)));

    /*
     *  now draw lidar rays using relative distances;
     *  furthest distance is largest line and all other lines use a % of the longest
     */

    double interval = 2 * M_PI / NUMPTS;
    double currAngle = 0;
    p.setPen(QPen(QColor(0, 0, 0, 32)));
    double mag;         // % of longest line

    for (int i = 0; i < NUMPTS; i++)
    {


        mag = dataPts[i] / range;
        if (mag > 1)
            mag = 1;
        p.drawLine(centerX, centerY,
                   centerX + cos(currAngle) * mag * minDimension * .5,
                   centerY - sin(currAngle) * mag * minDimension * .5);
        currAngle += interval;
    }
}

/*void LidarAdapter::OnLidarData(IGVC::Sensors::LidarState state)
{
    currState = state;
    hasState = true;
}*/

// TODO : Maybe add input textfield for range

void LidarAdapter::on_btn_fit_clicked()
{
    double maxSize = dataPts[0];
    for (int i = 1; i < NUMPTS; i++)    // find longest line
        if (dataPts[i] > maxSize)
            maxSize = dataPts[i];
    fitRange = maxSize;
    isFit = true;
    ui->btn_slider->setSliderPosition(fitRange);
    update();
}

void LidarAdapter::on_btn_slider_actionTriggered(int action)
{
    isFit = false;
    update();
}

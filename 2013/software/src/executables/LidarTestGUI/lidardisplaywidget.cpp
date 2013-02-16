#include "lidardisplaywidget.h"
#include "qpainter.h"

LidarDisplayWidget::LidarDisplayWidget(QWidget *parent) :
    QWidget(parent),
    LonNewLidarData(this)
{
    _lidar.onNewData += &LonNewLidarData;
    _scale = 1.0;
    _drawing = false;
}

void LidarDisplayWidget::paintEvent(QPaintEvent *)
{
    _drawing = true;
    QPainter painter;
    painter.begin(this);
    QPointF origin(this->width()/2.0, this->height()/2.0);
    boost::mutex::scoped_lock(_locker);
    for(int i = 0; i < 1024; i++)
    {
        LidarPoint p = _lidarData.points[i];
        if(p.valid)
        {
            QPointF end(cos(p.angle)*p.distance*_scale + origin.x(), sin(p.angle)*p.distance*_scale + origin.y());
//            std::cout << end.x() << ", " << end.y() << std::endl;
            //painter.drawLine(origin, end);
            painter.drawPoint(end);
        }
    }
    painter.end();
    _drawing = false;
}

void LidarDisplayWidget::onNewLidarData(LidarState state)
{
    while(_drawing);
    boost::mutex::scoped_lock(_locker);
    _lidarData = state;
    repaint();
}

void LidarDisplayWidget::setScale(double scale)
{
    _scale = scale;
}

double LidarDisplayWidget::scale()
{
    return _scale;
}

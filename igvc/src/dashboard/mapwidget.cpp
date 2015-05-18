#include "mapwidget.h"
#include <QPainter>

using namespace std;
using namespace pcl;

MapWidget::MapWidget(QWidget *parent)
    : QWidget(parent)
{
    map = PointCloud<PointXYZ>().makeShared();
}

void MapWidget::setMap(PointCloud<PointXYZ>::ConstPtr value)
{
    *map = *value;
    update();
}

void MapWidget::paintEvent(QPaintEvent *)
{
    if(map->empty())
    {
        return;
    }

    float minx = map->at(0).x;
    float maxx = map->at(0).x;
    float miny = map->at(0).y;
    float maxy = map->at(0).y;
    for(auto point : *map)
    {
        minx = min(minx, point.x);
        maxx = max(maxx, point.x);
        miny = min(miny, point.y);
        maxy = max(maxy, point.y);
    }

    auto viewWidth = maxx - minx;
    auto viewHeight = maxy - miny;

    QPainter painter(this);
    painter.setPen(Qt::black);
    for(auto point : *map)
    {
        // Draw point
        auto x = (point.x - minx) / viewWidth;
        auto y = (point.y - miny) / viewHeight;
        painter.drawEllipse(QPointF(x,y), 3, 3);
    }
}

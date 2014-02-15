#include "positiontrackeradapter.h"
#include "ui_positiontrackeradapter.h"
#include <QPainter>
#include <cmath>

PositionTrackerAdapter::PositionTrackerAdapter(Event<RobotPosition> *src, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PositionTrackerAdapter),
    source(src),
    minx(-0.5),
    maxx( 0.5),
    miny(-0.5),
    maxy( 0.5),
    LonNewPosition(this)
{
    ui->setupUi(this);

    if(source != nullptr)
        (*source) += &LonNewPosition;

    connect(this, SIGNAL(updateBecauseOfData()), this, SLOT(update()));
}

PositionTrackerAdapter::~PositionTrackerAdapter()
{
    if(source != nullptr)
        (*source) -= &LonNewPosition;
    positions.clear();
    delete ui;
}

void PositionTrackerAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    if(positions.size() > 0)
    {
        QPen pen(Qt::black);
        pen.setWidth(5);
        painter.setPen(pen);

        int margin = 4;
        int w = this->width() - margin*2;
        int h = this->height() - margin*2;

        double xscale = w / (maxx - minx);
        double yscale = h / (maxy - miny);

        int lastx = ( positions[0].X - minx ) * xscale;
        int lasty = h - ( ( positions[0].Y - miny ) * yscale );
        for(uint i = 1; i < positions.size(); i++)
        {
            RobotPosition p = positions[i];
            int x = ( p.X - minx ) * xscale;
            int y = h - ( ( p.Y - miny ) * yscale );
            painter.drawLine(lastx+margin, lasty+margin, x+margin, y+margin);
            lastx = x;
            lasty = y;
        }
    }

    painter.end();
}

void PositionTrackerAdapter::onNewPosition(RobotPosition pos)
{
    positions.push_back(pos);
    minx = std::min(pos.X, minx);
    maxx = std::max(pos.X, maxx);
    miny = std::min(pos.Y, miny);
    maxy = std::max(pos.Y, maxy);
    updateBecauseOfData();
}

void PositionTrackerAdapter::on_pushButton_clicked()
{
    positions.clear();
    minx = -0.5;
    maxx =  0.5;
    miny = -0.5;
    maxy =  0.5;
    updateBecauseOfData();
}

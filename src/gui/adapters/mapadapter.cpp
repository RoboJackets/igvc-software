#include "mapadapter.h"
#include "ui_mapadapter.h"
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>

using namespace pcl;

MapAdapter::MapAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapAdapter),
    _scale(20),
    _map(0),
    LonNewMap(this)
{
    ui->setupUi(this);
}

void MapAdapter::setScale(int scale)
{
    scale = std::min(scale, 100);
    scale = std::max(scale, 1);
    _scale = scale;
    ui->scaleSlider->setValue(scale);
    update();
}

void MapAdapter::onNewMap(pcl::PointCloud<PointXYZ> *map)
{
    _map = map;
    update();
}

void MapAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    // Draw axes
    painter.setPen(Qt::red);
    QPoint center = QPoint(this->width()/2, this->height()/2) + _origin;
    painter.drawLine(center, center + QPoint(0,-1*_scale));
    painter.setPen(Qt::blue);
    painter.drawLine(center, center + QPoint(1*_scale,0));

    // Draw map
    if(_map != NULL)
    {
        if(_map->size() == 0)
        {
            painter.setPen(Qt::red);
            painter.drawText(QPoint(this->width()/2-40,this->height()/2), tr("EMPTY MAP"));
        }
        else
        {
            painter.setPen(Qt::black);
            PointCloud<PointXYZ>::iterator iter;
            for(iter = _map->begin(); iter != _map->end(); iter++)
            {
                PointXYZ mPoint = (*iter);
                QPoint sPoint = QPoint((int)(mPoint.x * _scale), (int)(mPoint.y * _scale));
                sPoint += center;
                // Displays points as 3x3 pixel crosses
                painter.drawLine(sPoint - QPoint(1,0), sPoint + QPoint(1,0));
                painter.drawLine(sPoint - QPoint(0,1), sPoint + QPoint(0,1));
            }
        }
    }
    else
    {
        painter.setPen(Qt::red);
        painter.drawText(QPoint(this->width()/2-25,this->height()/2), tr("NO MAP"));
    }

    painter.end();
}

MapAdapter::~MapAdapter()
{
    delete ui;
}

void MapAdapter::on_scaleSlider_sliderMoved(int position)
{
    setScale(position);
}

void MapAdapter::on_homeButton_clicked()
{
    _origin = QPoint(0,0);
    setScale(20);
}

void MapAdapter::mousePressEvent(QMouseEvent *e)
{
    _previousMousePosition = e->pos();
}

void MapAdapter::mouseMoveEvent(QMouseEvent *e)
{
    if(e->buttons() & Qt::LeftButton)
    {
        QPoint delta = e->pos() - _previousMousePosition;
        _origin += delta;
        _previousMousePosition = e->pos();
        update();
    }
}

void MapAdapter::wheelEvent(QWheelEvent *e)
{
    setScale(_scale + ( e->angleDelta().y() / 30 ) );
}

#include "systemstatusindicator.h"
#include <QtGui>
#include <math.h>

SystemStatusIndicator::SystemStatusIndicator(QWidget *parent) :
    QWidget(parent)
{
}

void SystemStatusIndicator::paintEvent(QPaintEvent *)
{
    QPainter qp(this);

    QColor color(255,0,0);

    qp.setPen(color);
    qp.setBrush(color);

    int size = std::min(this->width(), this->height())-1;

    qp.drawEllipse(0,0,size, size);
}

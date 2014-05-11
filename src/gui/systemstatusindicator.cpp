#include "systemstatusindicator.h"
#include <QtGui>
#include <math.h>

SystemStatusIndicator::SystemStatusIndicator(QWidget *parent) :
    QWidget(parent)
{
    _isEnabled = false;
}

void SystemStatusIndicator::paintEvent(QPaintEvent *)
{
    QPainter qp(this);

    qp.setPen(_isEnabled ? Qt::green : Qt::red);
    qp.setBrush(_isEnabled ? Qt::green : Qt::red);

    int size = std::min(this->width(), this->height())-1;

    qp.drawEllipse(0,0,size, size);
}

void SystemStatusIndicator::onEStopStatusChanged(bool isEnabled)
{
    _isEnabled = isEnabled;
}

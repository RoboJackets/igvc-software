#include "systemstatusindicator.h"
#include <QtGui>
#include <math.h>
#include <common/logger/logger.h>

SystemStatusIndicator::SystemStatusIndicator(QWidget *parent) :
    QWidget(parent)
{
    _isEnabled = false;
    connect(this, SIGNAL(updateBecauseData()), this, SLOT(update()));
}

void SystemStatusIndicator::paintEvent(QPaintEvent *)
{
    _paintMutex.lock();
    QPainter qp(this);

    qp.setPen(_isEnabled ? Qt::green : Qt::red);
    qp.setBrush(_isEnabled ? Qt::green : Qt::red);

    int size = std::min(this->width(), this->height())-1;

    qp.drawEllipse(0,0,size, size);
    _paintMutex.unlock();
}

void SystemStatusIndicator::onEStopStatusChanged(bool isEnabled)
{
    _isEnabled = isEnabled;
    updateBecauseData();
}

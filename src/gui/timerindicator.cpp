#include "timerindicator.h"
#include <iostream>
#include <QTimer>
#include <QString>

TimerIndicator::TimerIndicator(QWidget* parent) : QLabel(parent)
{
    time(&startTime);
    qtimer = new QTimer(this);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(update()));
    qtimer->start(1000);
    this->setText("hi");
    this->setEnabled(true);
}

void TimerIndicator::paintEvent(QPaintEvent*)
{
    time_t timer;
    time(&timer);
    QString curTime = QString::number(difftime(timer, startTime));
    this->setText(curTime);
    std::cout << this->text().toStdString() << std::endl;
}

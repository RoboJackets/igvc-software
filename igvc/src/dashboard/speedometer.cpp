#include "speedometer.h"
#include <QPainter>
#include <QTimer>
#include <QTime>

#include <iostream>

using namespace std;

Speedometer::Speedometer(QWidget *parent) :
    QWidget(parent),
    value(0)
{
//    QTimer *timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
//    timer->start(100);

    resize(200,200);
}

void Speedometer::paintEvent(QPaintEvent *event)
{
    static const QPoint needle[4] = {
        QPoint(4,8),
        QPoint(-4,8),
        QPoint(-1,-90),
        QPoint(1,-90)
    };

    auto text = tr((to_string(value) + " m/s").c_str());

    QColor needleColor(0,0,0);

    auto side = qMin(width(), height());
    auto time = QTime::currentTime();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(width() / 2, height() / 2);
    painter.scale(side / 200.0, side / 200.0);

    painter.setPen(Qt::NoPen);
    painter.setBrush(needleColor);

    painter.save();
    painter.rotate( (value * (180./6.)) - 90.);

    painter.drawConvexPolygon(needle, 4);
    painter.restore();

    painter.setPen(Qt::black);
    auto text_width = painter.fontMetrics().width(text);
    painter.drawText(QRectF(-text_width/2., -40, 100, 20), text);

    for (int j = 0; j < 30; ++j) {
        painter.setPen(QColor(255-((255./30.)*j),(255./30.)*j,0));
        if ((j % 5) != 0)
            painter.drawLine(92, 0, 96, 0);
        else
            painter.drawLine(84, 0, 96, 0);
        painter.rotate(-6.0);
    }
}

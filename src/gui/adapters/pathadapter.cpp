#include "pathadapter.h"
#include "ui_pathadapter.h"
#include <QPainter>
#include <math.h>
#include <common/utils/AngleUtils.h>

PathAdapter::PathAdapter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PathAdapter)
{
    ui->setupUi(this);
}

PathAdapter::~PathAdapter()
{
    delete ui;
}

void PathAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    float pixPerM = 50;

    float xp = 10;
    float yp = 10;
    float tp = 0;

    float V = 1;
    float w = 2;
    float dt = 1;

    float R = (V/w) * pixPerM;

    int x = xp*pixPerM - R + R*cos(tp);

    int y = yp*pixPerM + R*sin(tp) - R;

    int a = AngleUtils::radsToDeg((M_PI - tp) - (w * dt)) * 16;

    int alen = AngleUtils::radsToDeg(w*dt) * 16;

    painter.setPen(Qt::black);
    painter.drawArc(x, y, 2*R, 2*R, a, alen);
    painter.setPen(Qt::red);
    painter.drawPoint(xp*pixPerM,yp*pixPerM);

    painter.end();
}

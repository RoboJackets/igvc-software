#include <igvc_rviz_plugins_old/speedometer.h>
#include <QPainter>
#include <QTime>
#include <QTimer>

#include <iostream>

using namespace std;

Speedometer::Speedometer(QWidget *parent) : QWidget(parent), value(0), maxVal(2.)
{
  resize(200, 125);
}

void Speedometer::setValue(float value)
{
  this->value = value;
  update();
}
void Speedometer::setMaxValue(float maxVal)
{
  this->maxVal = maxVal;
  update();
}
void Speedometer::paintEvent(QPaintEvent *event)
{
  static const QPoint needle[4] = { QPoint(4, 8), QPoint(-4, 8), QPoint(-1, -90), QPoint(1, -90) };

  auto text = tr((to_string(value) + " m/s").c_str());

  QColor needleColor(0, 0, 0);

  auto side = 200;
  // auto time = QTime::currentTime();

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.translate(width() / 2, height() - 25);
  painter.scale(side / 200.0, side / 200.0);

  painter.setPen(Qt::NoPen);
  painter.setBrush(needleColor);

  painter.save();
  painter.rotate((value * (180. / maxVal)) - 90.);

  painter.drawConvexPolygon(needle, 4);
  painter.restore();

  painter.setPen(Qt::black);
  auto text_width = painter.fontMetrics().width(text);
  painter.drawText(QRectF(-text_width / 2., -40, 100, 20), text);

  for (int j = 0; j < 30; ++j)
  {
    if ((j % 5) != 0)
    {
      painter.setPen(QColor(255 - ((255. / 30.) * j), (255. / 30.) * j, 0));
      painter.drawLine(92, 0, 96, 0);
    }
    else
    {
      painter.setPen(QColor(0, 0, 0));
      painter.drawLine(84, 0, 96, 0);
    }
    painter.rotate(-6.0);
  }
}

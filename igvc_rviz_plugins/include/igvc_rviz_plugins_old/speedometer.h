#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QWidget>

class Speedometer : public QWidget
{
  Q_OBJECT
public:
  explicit Speedometer(QWidget *parent = 0);
  void setValue(float value);
  void setMaxValue(float maxVal);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  float value;
  float maxVal;
};

#endif  // SPEEDOMETER_H

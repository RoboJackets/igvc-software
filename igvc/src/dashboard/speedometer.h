#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QWidget>

class Speedometer : public QWidget
{
    Q_OBJECT
public:
    explicit Speedometer(QWidget *parent = 0);

public slots:
    void setValue(float value);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    float value;
};

#endif // SPEEDOMETER_H

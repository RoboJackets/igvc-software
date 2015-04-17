#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QWidget>

class Speedometer : public QWidget
{
    Q_OBJECT
public:
    explicit Speedometer(QWidget *parent = 0);

    float value;

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

signals:

public slots:

};

#endif // SPEEDOMETER_H

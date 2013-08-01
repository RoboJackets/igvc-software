#ifndef BATTERYINDICATOR_H
#define BATTERYINDICATOR_H

#include <QProgressBar>

class BatteryIndicator : public QProgressBar
{
    Q_OBJECT
public:
    explicit BatteryIndicator(QWidget *parent = 0);

signals:
    
public slots:
    
};

#endif // BATTERYINDICATOR_H

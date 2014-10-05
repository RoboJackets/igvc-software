#ifndef TIMERINDICATOR_H
#define TIMERINDICATOR_H

#include <time.h>
#include <QLabel>

class TimerIndicator : public QLabel
{
    Q_OBJECT
public:
    explicit TimerIndicator(QWidget* parent = 0);

protected:
    void paintEvent(QPaintEvent *);

private:
    time_t startTime;
    QTimer* qtimer;
};

#endif // TIMERINDICATOR_H

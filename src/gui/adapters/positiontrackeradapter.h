#ifndef POSITIONTRACKERADAPTER_H
#define POSITIONTRACKERADAPTER_H

#include <QWidget>
#include <common/datastructures/robotposition.hpp>
#include <common/events/Event.hpp>
#include <vector>

namespace Ui {
class PositionTrackerAdapter;
}

class PositionTrackerAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit PositionTrackerAdapter(Event<RobotPosition> *src, QWidget *parent = 0);
    ~PositionTrackerAdapter();

protected:
    void paintEvent(QPaintEvent*);

Q_SIGNALS:
    void updateBecauseOfData();

private slots:
    void on_pushButton_clicked();

private:
    Ui::PositionTrackerAdapter *ui;

    Event<RobotPosition> *source;

    std::vector<RobotPosition> positions;

    double minx, maxx, miny, maxy;

    void onNewPosition(RobotPosition pos);
    LISTENER(PositionTrackerAdapter, onNewPosition, RobotPosition)
};

#endif // POSITIONTRACKERADAPTER_H

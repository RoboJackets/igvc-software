#ifndef COMPETITIONCONTROLLERADAPTER_H
#define COMPETITIONCONTROLLERADAPTER_H

#include <QWidget>
#include <intelligence/controller/controller.h>
#include <hardware/sensors/gps/GPS.hpp>

namespace Ui {
class CompetitionControllerAdapter;
}

class CompetitionControllerAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit CompetitionControllerAdapter(Controller *controller, GPS *gps, QWidget *parent = 0);
    ~CompetitionControllerAdapter();

public slots:
    void onNewGPS(GPSData data);

private:
    Ui::CompetitionControllerAdapter *ui;

    Controller *_compController;

    GPS *_gps;
};

#endif // COMPETITIONCONTROLLERADAPTER_H

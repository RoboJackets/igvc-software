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
    explicit CompetitionControllerAdapter(std::shared_ptr<Controller> controller, std::shared_ptr<GPS> gps, QWidget *parent = 0);
    ~CompetitionControllerAdapter();

public slots:
    void onNewGPS(GPSData data);

private:
    Ui::CompetitionControllerAdapter *ui;

    std::shared_ptr<Controller> _compController;

    std::shared_ptr<GPS> _gps;
};

#endif // COMPETITIONCONTROLLERADAPTER_H

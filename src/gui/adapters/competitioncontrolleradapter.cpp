#include "competitioncontrolleradapter.h"
#include "ui_competitioncontrolleradapter.h"

CompetitionControllerAdapter::CompetitionControllerAdapter(std::shared_ptr<Controller> controller, std::shared_ptr<GPS> gps, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CompetitionControllerAdapter)
{
    ui->setupUi(this);

    _compController = controller;
    _gps = gps;
    if(_gps.get())
        connect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void CompetitionControllerAdapter::onNewGPS(GPSData data)
{
    ui->label_currentPos->setText(tr("%1,%2").arg(data.Lat()).arg(data.Long()));
    ui->label_waypoint->setText(tr("%1,%2").arg(_compController->getCurrentWaypoint().Lat()).arg(_compController->getCurrentWaypoint().Long()));
}

CompetitionControllerAdapter::~CompetitionControllerAdapter()
{
    if(_gps.get())
        disconnect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    delete ui;
}

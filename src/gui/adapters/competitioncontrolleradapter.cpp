#include "competitioncontrolleradapter.h"
#include "ui_competitioncontrolleradapter.h"

CompetitionControllerAdapter::CompetitionControllerAdapter(Controller *controller, GPS *gps, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CompetitionControllerAdapter)
{
    ui->setupUi(this);

    _compController = controller;
    _gps = gps;
    if(_gps)
        connect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void CompetitionControllerAdapter::onNewGPS(GPSData data)
{
    ui->label_currentPos->setText(tr("%1,%2").arg(data.Lat()).arg(data.Long()));
    ui->label_waypoint->setText(tr("%1,%2").arg(_compController->getCurrentWaypoint().Lat()).arg(_compController->getCurrentWaypoint().Long()));
}

CompetitionControllerAdapter::~CompetitionControllerAdapter()
{
    if(_gps)
        disconnect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
    delete ui;
}

#include "gpsadapter.h"
#include "ui_gpsadapter.h"
#include <cmath>

GPSAdapter::GPSAdapter(std::shared_ptr<GPS> gps, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GPSAdapter)
{
    ui->setupUi(this);
    for (int i = 0;i<5;i++) {
        coordinates[i][0] = 0;
        coordinates[i][1] = 0;
    }

    _GPS = gps;
    if(_GPS.get() != nullptr)
        connect(_GPS.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewData(GPSData)));
}

GPSAdapter::~GPSAdapter()
{
    if(_GPS.get() != nullptr)
    {
        disconnect(_GPS.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewData(GPSData)));
        _GPS.reset();
    }
    delete ui;

}

void GPSAdapter::labelPrint() {
    //paint data label
    QString LatLabel = QString("Latitude<br />");
    for (int i = 0;i < 5;i++) {
        LatLabel = LatLabel + QString::number(coordinates[i][0],'g',15) + QString("<br />");
    }
    ui->LatitudeLabel->setText(LatLabel);

    QString LongLabel = QString("Longitude<br />");
    for (int i = 0;i < 5;i++) {
        LongLabel = LongLabel + QString::number(coordinates[i][1], 'g', 15) + QString("<br />");
    }
    ui->LongitudeLabel->setText(LongLabel);
}

void GPSAdapter::onNewData(GPSData data) {
    //shifting old data
    for (int i = 4;i>0;i--) {
        coordinates[i][0] = coordinates[i-1][0];
        coordinates[i][1] = coordinates[i-1][1];
    }

    //assign new data
    coordinates[0][0] = data.Lat();
    coordinates[0][1] = data.Long();

    //print data label
    labelPrint();
}

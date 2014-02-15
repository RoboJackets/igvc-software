#include "gpsadapter.h"
#include "ui_gpsadapter.h"
#include <cmath>

GPSAdapter::GPSAdapter(GPS *gps, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GPSAdapter),
    minLat(-5),
    maxLat(5),
    minLong(-5),
    maxLong(5),
    LOnNewData(this)
{
    ui->setupUi(this);
    for (int i = 0;i<5;i++) {
        coordinates[i][0] = 0;
        coordinates[i][1] = 0;
    }
    gps->onNewData += &LOnNewData;
    _GPS = gps;
    ui->user_Top->setPlainText(QString::number(maxLat));
    ui->user_Right->setPlainText(QString::number(maxLong));
    ui->user_Bottom->setPlainText(QString::number(minLat));
    ui->user_Left->setPlainText(QString::number(minLong));

    connect(this, SIGNAL(updateBecauseNewData()), this, SLOT(update()));
}

GPSAdapter::~GPSAdapter()
{
    if(_GPS != nullptr)
        _GPS->onNewData -= &LOnNewData;
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


    horizontalFactor = ui->GraphicsHolder->width() / (maxLong - minLong);
    verticalFactor = ui->GraphicsHolder->height() / (maxLat - minLat);

}

void GPSAdapter::OnNewData(GPSData data) {
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


    //update graphics holder
    updateBecauseNewData();
}



void GPSAdapter::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    painter.setPen(Qt::black);

    //black ackground painting
    painter.fillRect(ui->GraphicsHolder->x(),
                     ui->GraphicsHolder->y(),
                     ui->GraphicsHolder->width(),
                     ui->GraphicsHolder->height(),
                     Qt::SolidPattern);


    //draw X and Y axis
    //X axis: longitude, Y axis:latitude
    painter.setPen(Qt::white);
    painter.drawLine(QPoint(ui->GraphicsHolder->x()+ui->GraphicsHolder->width()/2,ui->GraphicsHolder->y()),
                     QPoint(ui->GraphicsHolder->x()+ui->GraphicsHolder->width()/2,ui->GraphicsHolder->y()+ui->GraphicsHolder->height())); //y axis
    painter.drawLine(QPoint(ui->GraphicsHolder->x(),ui->GraphicsHolder->y()+ui->GraphicsHolder->height()/2),
                     QPoint(ui->GraphicsHolder->x()+ui->GraphicsHolder->width(),ui->GraphicsHolder->y()+ui->GraphicsHolder->height()/2)); //x axis

    //draw the latest five points
    //biggest point: the lattest
    painter.setBrush(Qt::red);
    QPoint origin = QPoint(ui->GraphicsHolder->x() + ui->GraphicsHolder->width()/2,
                           ui->GraphicsHolder->y() + ui->GraphicsHolder->height()/2);
    painter.drawEllipse(origin, 6, 6);
    for (int i = 0; i < 4;i++) {
        painter.drawEllipse(origin + QPoint((coordinates[i+1][1] - coordinates[0][1]) * horizontalFactor,
                - (coordinates[0][0] - coordinates[i+1][0]) * verticalFactor), 5 - i, 5 - i);
    }

    painter.end();
}

void GPSAdapter::on_user_Top_textChanged()
{
    maxLat = QString(ui->user_Top->toPlainText()).toDouble();
}

void GPSAdapter::on_user_Right_textChanged()
{
    maxLong = QString(ui->user_Right->toPlainText()).toDouble();
}

void GPSAdapter::on_user_Bottom_textChanged()
{
    minLat = QString(ui->user_Bottom->toPlainText()).toDouble();
}

void GPSAdapter::on_user_Left_textChanged()
{
    minLong = QString(ui->user_Left->toPlainText()).toDouble();
}

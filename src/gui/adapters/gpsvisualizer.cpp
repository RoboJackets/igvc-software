#include "gpsvisualizer.h"
#include "ui_gpsvisualizer.h"
#include <cmath>

GPSVisualizer::GPSVisualizer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GPSVisualizer)

{
    ui->setupUi(this);
    scene = new QGraphicsScene();
    for (int i = 0;i<5;i++) {

            coordinates[i][0] = 0;
            coordinates[i][1] = 0;

    }



}

void GPSVisualizer::sceneConst() {

    pixmap= QPixmap(QSize(ui->sizeHint()));\
    painter.begin(&pixmap);
    painter.setPen(Qt::white);
    painter.drawLine(QPoint(0,pixmap.height()/2),QPoint(pixmap.width(),pixmap.height()/2));
    painter.drawLine(QPoint(pixmap.width()/2,0),QPoint(pixmap.width()/2,pixmap.height()));
}

GPSVisualizer::~GPSVisualizer()
{
    delete ui;
    painter.end();
}

void GPSVisualizer::labelPrint() {
    ui->dataLabel->setText(QString("Latitude                Longitude\n"));
    for (int i=0;i<5;i++) {

        ui->dataLabel->setText(QString::number(coordinates[i][0]));
        ui->dataLabel->setText(QString('                '));
        ui->dataLabel->setText(QString::number(coordinates[i][1]));
        ui->dataLabel->setText(QString('\n'));

    }
}

void GPSVisualizer::onNewData(GPSData& data) {
    for (int i = 0;i<5;i++) {

            coordinates[i+1][0] = coordinates[i][0];
            coordinates[i+1][1] = coordinates[i][1];
            //first index:points from most recent[0] to least recent;
            //second index: [0]->latitude,[1]->longitude
    }

    coordinates[0][0] = data.Lat();
    coordinates[0][1] = data.Long();
    labelPrint();
    double minLat = coordinates[0][0];
    double maxLat = minLat;
    double minLong = coordinates[0][1];
    double maxLong = minLong;
    for (int i=0;i<5;i++) {
        if (coordinates[i][0] <= minLat) {
            minLat = coordinates[i][0];
        }
        else {
            maxLat = coordinates[i][0];
        }
        if (coordinates[i][1] <= minLong) {
            minLong = coordinates[i][1];
        }
        else {
            maxLong = coordinates[i][1];
        }
    }
    horizontalFactor = (maxLong - minLong) / QSize(ui->graphics->width());
    verticalFactor = (maxLat - minLat) / QSize(ui->graphics->height());
}

void GPSVisualizer::boundaryLabelPrint() {
    painter.setPen(Qt::black);
    painter.drawText(QPoint(ui->graphics->width()/2,ui->graphics->height()), QString::number(minLat));//bottom
    painter.drawText(QPoint(ui->graphics->width(),0), QString::number(maxLat));//top
    painter.rotate(270);
    painter.drawText(QPoint(ui->graphics->width(),0), QString::number(minLong));//left
    painter.rotate(180);
    painter.drawText(QPoint(ui->graphics->width(),0), QString::number(maxLong));//right
    painter.rotate(270);
}

void GPSVisualizer::drawPoints() {
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPoint(10,10),5,5);
}

void GPSVisualizer::paintEvent(QPaintEvent *event)
{

    sceneConst();
    boundaryLabelPrint();
    drawPoints();
    scene->addPixmap(pixmap);
    ui->graphics->setScene(scene);

}

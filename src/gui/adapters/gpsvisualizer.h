#ifndef GPSVISUALIZER_H
#define GPSVISUALIZER_H

#include <QWidget>
#include <QString>
#include <QPixmap>
#include <QPainter>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLabel>
#include <hardware/sensors/gps/GPS.hpp>

using namespace IGVC::Sensors;

namespace Ui {
class GPSVisualizer;
}

class GPSVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit GPSVisualizer(GPS *gps, QWidget *parent = 0);
    void labelPrint();
    void paintEvent(QPaintEvent *event);
    ~GPSVisualizer();

private slots:
    void on_user_Top_textChanged();

    void on_user_Right_textChanged();

    void on_user_Bottom_textChanged();

    void on_user_Left_textChanged();

private:

    Ui::GPSVisualizer *ui;
    //QGraphicsScene* scene;
    //QPixmap pixmap;
    double coordinates[5][2];
    QLabel dataLabel;
    double horizontalFactor;
    double verticalFactor;
    double minLat;
    double maxLat;
    double minLong;
    double maxLong;
    //first index:points from most recent[0] to least recent;
    //second index: [0]->latitude,[1]->longitude

    void OnNewData(GPSData data);
    LISTENER(GPSVisualizer,OnNewData,GPSData)
    GPS* _GPS;
};


#endif // GPSVISUALIZER_H

#ifndef GPSVISUALIZER_H
#define GPSVISUALIZER_H

#include <QWidget>
#include <QString>
#include <QPixmap>
#include <QPainter>
#include <QGraphicsView>
#include <QGraphicsScene>


namespace Ui {
class GPSVisualizer;
}

class GPSVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit GPSVisualizer(QWidget *parent = 0);
    void sceneConst();
    void labelPrint();
    void boundaryLabelPrint();
    void drawPoints();
    void onNewData(GPSData& );
    void paintEvent(QPaintEvent *event);
    ~GPSVisualizer();

private:
    Ui::GPSVisualizer *ui;
    QGraphicsScene* scene;
    QPixmap pixmap;
    QPainter painter;
    double coordinates[5][2];
    QLabel dataLabel;
    double horizontalFactor;
    double verticalFactor;
    //first index:points from most recent[0] to least recent;
    //second index: [0]->latitude,[1]->longitude

};


#endif // GPSVISUALIZER_H

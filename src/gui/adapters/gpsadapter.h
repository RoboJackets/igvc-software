#ifndef GPSVISUALIZER_H
#define GPSVISUALIZER_H

#include <QWidget>
#include <QString>
#include <QPainter>
#include <QLabel>
#include <hardware/sensors/gps/GPS.hpp>
#include <memory>


namespace Ui {
class GPSAdapter;
}

/*!
 * \brief Widget for displaying GPS data.
 * \author Victor Ying
 */
class GPSAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit GPSAdapter(std::shared_ptr<GPS> gps, QWidget *parent = 0);
    void labelPrint();
    ~GPSAdapter();

private slots:
    void onNewData(GPSData data);

private:

    Ui::GPSAdapter *ui;

    /*
     * First index identifies coordinates (0 -> n, newest -> oldest)
     * Second index identifies parts of coordinates (0=latitude, 1=longitude)
     */
    double coordinates[5][2];
    QLabel dataLabel;

    GPS_QUALITY quality;
    int numSats;
    float hdop;

    std::shared_ptr<GPS> _GPS;
};


#endif // GPSVISUALIZER_H

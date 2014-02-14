#ifndef CAPTUREGPSANDPOSTRACKER_HPP
#define CAPTUREGPSANDPOSTRACKER_HPP

#include <QtTest>
#include <hardware/sensors/gps/nmeacompatiblegps.h>
#include <intelligence/posetracking/basicpositiontracker.h>
#include <QFile>
#include <QTextStream>

class CaptureGPSAndPosTracker : public QObject
{
    Q_OBJECT

public:
    CaptureGPSAndPosTracker(QObject *parent = 0)
        : QObject(parent),
          LonNewPosData(this),
          LonNewGPSData(this),
          gps("/dev/ttyGPS", 19200)
    {
        gpsPts = 0;
        ptPts = 0;
        PT_PTS = 1800;
        GPS_PTS = PT_PTS + 300;
    }

    ~CaptureGPSAndPosTracker()
    {
        delete GPSOut;
        delete gpsfile;
        delete PTOut;
        delete ptfile;
    }

private:
    void onNewPosData(RobotPosition pos)
    {
        (*PTOut) << pos.X << "," << pos.Y << "," << pos.Heading << "\n";
        ptPts++;
        if(ptPts > PT_PTS)
        {
            tracker.onNewPosition -= &LonNewPosData;
            ptfile->close();
        }
    }
    LISTENER(CaptureGPSAndPosTracker, onNewPosData, RobotPosition)

    void onNewGPSData(GPSData d)
    {
        (*GPSOut) << d.Lat() << "," << d.Long() << "\n";
        gpsPts++;
        if(gpsPts > GPS_PTS)
        {
            gps.onNewData -= &LonNewGPSData;
            gpsfile->close();
        }
    }
    LISTENER(CaptureGPSAndPosTracker, onNewGPSData, GPSData)

    QTextStream *GPSOut;
    QTextStream *PTOut;
    QFile *gpsfile;
    QFile *ptfile;

    int PT_PTS;
    int GPS_PTS;
    int gpsPts;
    int ptPts;

    NMEACompatibleGPS gps;
    BasicPositionTracker tracker;

private Q_SLOTS:
    void run()
    {
        gpsfile = new QFile("gps.csv");
        ptfile = new QFile("pt.csv");
        gpsfile->open(QIODevice::WriteOnly | QIODevice::Text);
        ptfile->open(QIODevice::WriteOnly | QIODevice::Text);
        GPSOut = new QTextStream(gpsfile);
        GPSOut->setRealNumberPrecision(15);
        PTOut = new QTextStream(ptfile);
        PTOut->setRealNumberPrecision(15);

        gps.onNewData += &(tracker.LonNewGPS);
        gps.onNewData += &LonNewGPSData;
        tracker.onNewPosition += &LonNewPosData;

        while(ptPts <= PT_PTS || gpsPts <= GPS_PTS)
        {
            usleep(500);
        }
    }
};

#endif // CAPTUREGPSANDPOSTRACKER_HPP

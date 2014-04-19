#ifndef CAPTUREIMU_HPP
#define CAPTUREIMU_HPP


#include <QtTest>
#include <hardware/sensors/IMU/Ardupilot.h>
#include <QFile>
#include <QTextStream>

class CaptureIMU : public QObject
{
    Q_OBJECT

public:
    CaptureIMU(QObject *parent = 0)
        : QObject(parent)
    {
    }

    ~CaptureIMU()
    {
        ptfile->close();
    }

private slots:
    void onNewIMU(IMUData data)
    {
        (*PTOut) << data.Yaw << "\n";
    }

private:
    QTextStream *PTOut;
    QFile *ptfile;

    Ardupilot imu;

private Q_SLOTS:
    void run()
    {
        ptfile = new QFile("pt.csv");
        ptfile->open(QIODevice::WriteOnly | QIODevice::Text);
        PTOut = new QTextStream(ptfile);
        PTOut->setRealNumberPrecision(15);

        connect(&imu, SIGNAL(onNewData(IMUData)), this, SLOT(onNewIMU(IMUData)));
        {
            usleep(500);
        }
    }
};

#endif // CAPTUREIMU_HPP

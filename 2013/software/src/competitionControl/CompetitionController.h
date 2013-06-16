#ifndef COMPETITIONCONTROLLER_H
#define COMPETITIONCONTROLLER_H

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <IGVC.hpp>
#include <sensors/GPS/GPS.hpp>
#include <competitionControl/WaypointReader.h>
#include <sensors/camera3D/StereoSource.hpp>
#include <sensors/lidar/Lidar.h>
#include <sensors/IMU/IMU.h>
#include <vector>
#include <actuators/motors/MotorDriver/MotorDriver.hpp>
#include <common/Robot.h>
#include <fstream>
#include <sensors/RobotPosition.h>

namespace IGVC
{
namespace Control
{

class CompetitionController
{
    public:
        CompetitionController(IGVC::Sensors::GPS* gps,
                              IMU* imu,
                              Event<pcl::PointCloud<pcl::PointXYZ> >* mapSource,
                              WaypointReader* waypointReader,
                              MotorDriver* driver,
                              string fileName = "/home/robojackets/Desktop/directionLog.txt");
        virtual ~CompetitionController();
        bool isRunning();
        double MaxW;
        double DeltaT;

    protected:
    private:
        bool _running;
        std::ofstream _logFile;
        RobotPosition _poseTracker;
        WaypointReader* _waypointReader;
        MotorDriver* _driver;
        std::vector<GPSData> _gpsBuffer;
        const unsigned int GPS_BUFFER_SIZE;
        GPSData _currentAvgGPS;
        double _currentHeading;
        bool _hasAllData;
        pcl::visualization::PCLVisualizer _viewer;

        void OnNewGPSData(GPSData data);
        LISTENER(CompetitionController, OnNewGPSData, GPSData);

        void OnNewMapFrame(pcl::PointCloud<pcl::PointXYZ>  mapFrame);
        LISTENER(CompetitionController, OnNewMapFrame, pcl::PointCloud<pcl::PointXYZ> );

        double headingFromAToB(GPSData A, GPSData B);

        double distBetween(GPSData A, GPSData B);
        double distBetween(pair<double, double> A, pair<double, double> B);
        double GPSdX(GPSData A, GPSData B);
        double GPSdY(GPSData A, GPSData B);

        double weighting(double delta);

        pair<double, double> result(double W, double V);
        pair<double, double> result(double W, double V, double dt);
};

}
}



#endif // COMPETITIONCONTROLLER_H

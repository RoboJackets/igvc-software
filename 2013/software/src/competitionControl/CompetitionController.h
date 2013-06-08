#ifndef COMPETITIONCONTROLLER_H
#define COMPETITIONCONTROLLER_H

#include <IGVC.hpp>
#include <sensors/GPS/GPS.hpp>
#include <competitionControl/WaypointReader.h>
#include <sensors/camera3D/StereoSource.hpp>
#include <sensors/lidar/Lidar.h>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <vector>

namespace IGVC
{
namespace Control
{

enum Mode
{
    ObstacleAvoidance,
    WaypointNavigation
};

class CompetitionController
{
    public:
        CompetitionController(IGVC::Sensors::GPS* gps,
                              IGVC::Sensors::Lidar* lidar,
                              StereoSource* stereoCam,
                              Ardupilot* imu,
                              WaypointReader* waypointReader);
        virtual ~CompetitionController();
        bool isRunning();
    protected:
    private:
        IGVC::Sensors::GPS* _gps;
        IGVC::Sensors::Lidar* _lidar;
        StereoSource* _stereoCam;
        Ardupilot* _imu;
        WaypointReader* _waypointReader;
        std::vector<GPSData> _gpsBuffer;
        const unsigned int GPS_BUFFER_SIZE;
        GPSData _currentAvgGPS;
        Mode _mode;
        double _currentHeading;

        void OnNewGPSData(GPSData data);
        LISTENER(CompetitionController, OnNewGPSData, GPSData);

        void OnNewIMUData(IMURawData data);
        LISTENER(CompetitionController, OnNewIMUData, IMURawData);

        double headingFromAToB(GPSData A, GPSData B);
};

}
}



#endif // COMPETITIONCONTROLLER_H

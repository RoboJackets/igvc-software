#ifndef COMPETITIONCONTROLLER_H
#define COMPETITIONCONTROLLER_H

#include <IGVC.hpp>
#include <sensors/GPS/GPS.hpp>
#include <competitionControl/WaypointReader.h>
#include <sensors/camera3D/StereoSource.hpp>
#include <sensors/lidar/Lidar.h>
#include <sensors/ardupilot/Ardupilot.hpp>
#include <vector>
#include <pcl/common/common_headers.h>

namespace IGVC
{
namespace Control
{

class CompetitionController
{
    public:
        CompetitionController(IGVC::Sensors::GPS* gps,
                              Event<pcl::PointCloud<PointXYZ> > mapSource,
                              Ardupilot* imu,
                              WaypointReader* waypointReader);
        virtual ~CompetitionController();
        bool isRunning();

        double MaxW;
        double DeltaT;

    protected:
    private:
        IGVC::Sensors::GPS* _gps;
        Ardupilot* _imu;
        WaypointReader* _waypointReader;
        std::vector<GPSData> _gpsBuffer;
        const unsigned int GPS_BUFFER_SIZE;
        GPSData _currentAvgGPS;
        double _currentHeading;

        void OnNewGPSData(GPSData data);
        LISTENER(CompetitionController, OnNewGPSData, GPSData);

        void OnNewIMUData(IMURawData data);
        LISTENER(CompetitionController, OnNewIMUData, IMURawData);

        void OnNewMapFrame(pcl::PointCloud<PointXYZ>  mapFrame);
        LISTENER(CompetitionController, OnNewMapFrame, pcl::PointCloud<PointXYZ> );

        double headingFromAToB(GPSData A, GPSData B);

        double distBetween(GPSData A, GPSData B);
        double GPSdX(GPSData A, GPSData B);
        double GPSdY(GPSData A, GPSData B);

        pair<double, double> result(double W, double V);
};

}
}



#endif // COMPETITIONCONTROLLER_H

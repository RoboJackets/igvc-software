#ifndef COMPETITIONCONTROLLER_H
#define COMPETITIONCONTROLLER_H

#include <sensors/GPS/GPS.hpp>
#include <competitionControl/WaypointReader.h>
#include <sensors/camera3D/StereoSource.hpp>
#include <sensors/lidar/Lidar.h>
#include <vector>

class CompetitionController
{
    public:
        CompetitionController(IGVC::Sensors::GPS* gps,
                              WaypointReader* waypointReader);
        virtual ~CompetitionController();
    protected:
    private:
        IGVC::Sensors::GPS* _gps;
        WaypointReader* _waypointReader;
        std::vector<GPSData> _gpsBuffer;

        void OnNewGPSData(GPSData data);
        LISTENER(CompetitionController, OnNewGPSData, GPSData);
};

#endif // COMPETITIONCONTROLLER_H

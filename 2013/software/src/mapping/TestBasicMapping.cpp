#include <iostream>
#include <pose_tracking/PoseTracker.h>
#include <sensors/lidar/NAV200.h>
#include <sensors/GPS/HemisphereA100GPS.h>
#include <sensors/ardupilot/Ardupilot.hpp>

#include <pcl/visualization/cloud_viewer.h>
GPSAccuracy IMUData::ArduPilotDefault= GPSAccuracy(.001, .001, 1, .1);

using namespace pcl;


pcl::visualization::CloudViewer viewer("Simple cloud viewer");

class BasicMapper {
public:
    BasicMapper(PoseTracker* tracker, Lidar* lidar) :
        LonNewPose(this),
        LonNewLidarFrame(this)
    {
        tracker->onNewPose += &LonNewPose;
        lidar->onNewData += &LonNewLidarFrame;
        _isFirstPose = true;
    }
private:
    PointCloud<PointXYZ> _map;
    Pose _pose;
    Pose _origin;
    bool _isFirstPose;
    void onNewPose(Pose pose) {
        _pose = pose;
        if(_isFirstPose)
        {
            _origin = pose;
            _isFirstPose = false;
        }
    }
    LISTENER(BasicMapper, onNewPose, Pose);

    void onNewLidarFrame(LidarState data) {
        double offx = (_pose.lon - _origin.lon) / 0.000010725;
        double offy = (_pose.lat - _origin.lat) / 0.000008997;

        for(int i = 0; i < 1024; i++)
        {
            LidarPoint point = data.points[i];
            if(point.valid)
            {
                if(true /*filter points that are the robot*/)
                {
                    double angle = point.angle + _pose.heading;
                    double x = point.distance * cos(angle) + offx;
                    double y = point.distance * sin(angle) + offy;
                    _map.points.push_back(PointXYZ(x, y, 0));

                }
            }
        }
        viewer.showCloud(_map.makeShared());
    }
    LISTENER(BasicMapper, onNewLidarFrame, LidarState);
};

int main() {
    HemisphereA100GPS GPS;
    Ardupilot IMU;
    NAV200 lidar;

    PoseTracker tracker(&GPS, &IMU);

    BasicMapper mapper(&tracker, &lidar);

    while(!viewer.wasStopped());
}

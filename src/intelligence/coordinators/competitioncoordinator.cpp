#include "competitioncoordinator.h"
#include <common/utils/GPSWaypointSource.h>
#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/sensors/camera/StereoSource.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <hardware/sensors/lidar/Lidar.h>
#include <intelligence/posetracking/kalmanpositiontracker.h>
#include <intelligence/mapping/mapbuilder.h>
#include <intelligence/pathplanning/astarplanner.h>
#include <intelligence/linedetection/linedetector.h>
#include <intelligence/controller/controller.h>
#include <intelligence/pathfollowing/pathfollower.h>
#include <intelligence/barrelfinder/barrelfinder.h>
#include <hardware/sensors/gps/simulatedgps.h>
#include <hardware/sensors/gps/nmeacompatiblegps.h>
#include <hardware/sensors/camera/StereoPlayback.h>
#include <hardware/sensors/camera/StereoImageRepeater.h>
#include <hardware/sensors/camera/Bumblebee2.h>
#include <hardware/sensors/IMU/Ardupilot.h>
#include <hardware/sensors/lidar/SimulatedLidar.h>
#include <hardware/sensors/lidar/lms200.h>
#include <common/utils/GPSUtils.h>

using namespace std;

CompetitionCoordinator::CompetitionCoordinator() {
    shared_ptr<GPS> gps = make_shared<NMEACompatibleGPS>("/dev/igvc_gps", 19200);
    shared_ptr<StereoSource> camera = make_shared<Bumblebee2>("../src/hardware/sensors/camera/calib/out_camera_data.xml");
    shared_ptr<IMU> imu = make_shared<Ardupilot>();
    shared_ptr<Lidar> lidar = make_shared<LMS200>();
    shared_ptr<PositionTracker> posTracker = make_shared<KalmanPositionTracker>(imu, gps);
    shared_ptr<MapBuilder> mapBuilder = make_shared<MapBuilder>(lidar, posTracker);
    shared_ptr<AStarPlanner> planner = make_shared<AStarPlanner>();
    shared_ptr<LineDetector> lineDetector = make_shared<LineDetector>();
    shared_ptr<BarrelFinder> barrelFinder = make_shared<BarrelFinder>();
    shared_ptr<GPSWaypointSource> waypointSource = make_shared<GPSWaypointSource>("");
    shared_ptr<Controller> controller = make_shared<Controller>(waypointSource, gps);
    shared_ptr<PathFollower> pathFollower = make_shared<PathFollower>();

    QObject::connect(camera.get(), SIGNAL(onNewLeftImage(ImageData)), barrelFinder.get(), SLOT(onNewImage(ImageData)));
    QObject::connect(camera.get(), SIGNAL(onNewLeftImage(ImageData)), lineDetector.get(), SLOT(onImageEvent(ImageData)));
    QObject::connect(lidar.get(), SIGNAL(onNewData(LidarState)), mapBuilder.get(), SLOT(onLidarData(LidarState)));
    QObject::connect(lineDetector.get(), SIGNAL(onNewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)), mapBuilder.get(), SLOT(onCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)));
    QObject::connect(barrelFinder.get(), SIGNAL(newCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXY)), mapBuilder.get(), SLOT(onCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)));

    QObject::connect(controller.get(), &Controller::onNewWaypoint, [=](GPSData waypoint){
        RobotPosition pos;
        auto trackerOrigin = posTracker->GetOrigin();
        GPSUtils::coordsToMetricXY(trackerOrigin.Lat(), trackerOrigin.Long(), waypoint.Lat(), waypoint.Long(), pos.X, pos.Y);
        planner->OnNewGoalPos(pos);
    });

    QObject::connect(posTracker.get(), SIGNAL(onNewPosition(RobotPosition)), planner.get(), SLOT(OnNewStartPos(RobotPosition)));

    QObject::connect(mapBuilder.get(), SIGNAL(onNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)), planner.get(), SLOT(OnNewMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

    QObject::connect(planner.get(), SIGNAL(OnNewPath(path_t)), pathFollower.get(), SLOT(onNewPath(path_t)));

    QObject::connect(pathFollower.get(), &PathFollower::newMotorCommand, [=](MotorCommand cmd) {
        emit newMotorCommand(cmd);
    });

    QObject::connect(controller.get(), &Controller::onNewWaypoint, [=](GPSData data){
        if(data.Lat() == 0 && data.Long() == 0) {
            emit finished();
        }
    });

    modules.push_back(gps);
    modules.push_back(camera);
    modules.push_back(imu);
    modules.push_back(lidar);
    modules.push_back(posTracker);
    modules.push_back(mapBuilder);
    modules.push_back(planner);
    modules.push_back(lineDetector);
    modules.push_back(barrelFinder);
    modules.push_back(controller);
    modules.push_back(pathFollower);
    modules.push_back(waypointSource);
}

CompetitionCoordinator::~CompetitionCoordinator() {
    modules.clear();
}

const module_list_t &CompetitionCoordinator::getModules() {
    return modules;
}

shared_ptr<Module> CompetitionCoordinator::getModuleWithName(string name) {
    for(auto module : modules) {
        if(module->moduleName() == name)
            return module;
    }
    return shared_ptr<Module>(0);
}

void CompetitionCoordinator::changeLidar(std::shared_ptr<Module> lidar) {
    auto pos = find("LIDAR");
    if(pos != modules.end())
        modules.erase(pos);
    modules.push_back(lidar);
    auto mapperPos = find("MapBuilder");
    if(mapperPos != modules.end()) {
        shared_ptr<MapBuilder> mapper = dynamic_pointer_cast<MapBuilder>(*mapperPos);
        mapper->ChangeLidar(dynamic_pointer_cast<Lidar>(lidar));
        mapper->Clear();
    }
}
void CompetitionCoordinator::changeGPS(std::shared_ptr<Module> gps) {
    auto pos = find("GPS");
    auto trackerPos = find("PositionTracker");
    if(pos != modules.end()) {
        if(trackerPos != modules.end()) {
            QObject::disconnect(dynamic_pointer_cast<GPS>(*pos).get(), SIGNAL(onNewData(GPSData)),
                                dynamic_pointer_cast<PositionTracker>(*trackerPos).get(), SLOT(onGPSData(GPSData)));
        }
        modules.erase(pos);
    }
    modules.push_back(gps);
    /*
     * Adding and removing modules from the modules
     * list changes the index of the PositionTracker
     * module, so we need to find its new position.
     */
    trackerPos = find("PositionTracker");
    if(trackerPos != modules.end()) {
        shared_ptr<PositionTracker> tracker = dynamic_pointer_cast<PositionTracker>(*trackerPos);
        QObject::connect(dynamic_pointer_cast<GPS>(gps).get(), SIGNAL(onNewData(GPSData)), tracker.get(), SLOT(onGPSData(GPSData)));
    }
}

module_list_t::iterator CompetitionCoordinator::find(string name) {
    for(auto iter = modules.begin(); iter != modules.end(); iter++) {
        if((*iter)->moduleName() == name)
            return iter;
    }
    return modules.end();
}

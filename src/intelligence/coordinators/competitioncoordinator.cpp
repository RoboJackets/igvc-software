#include "competitioncoordinator.h"
#include <common/utils/GPSWaypointSource.h>
#include <hardware/sensors/gps/GPS.hpp>
#include <hardware/sensors/camera/StereoSource.hpp>
#include <hardware/sensors/IMU/IMU.h>
#include <hardware/sensors/lidar/Lidar.h>
#include <intelligence/posetracking/basicpositiontracker.h>
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

using namespace std;

CompetitionCoordinator::CompetitionCoordinator() {
    shared_ptr<GPS> gps(new NMEACompatibleGPS("/dev/igvc_gps", 19200));
    shared_ptr<StereoSource> camera(new Bumblebee2());
    shared_ptr<IMU> imu(new Ardupilot());
    shared_ptr<Lidar> lidar(new LMS200());
    shared_ptr<BasicPositionTracker> posTracker(new BasicPositionTracker(gps, imu));
    shared_ptr<MapBuilder> mapBuilder(new MapBuilder(lidar, posTracker));
    shared_ptr<AStarPlanner> planner(new AStarPlanner());
    shared_ptr<LineDetector> lineDetector(new LineDetector());
    shared_ptr<BarrelFinder> barrelFinder(new BarrelFinder());
    shared_ptr<GPSWaypointSource> waypointSource(new GPSWaypointSource(""));
    shared_ptr<Controller> controller(new Controller(waypointSource, gps));
    shared_ptr<PathFollower> pathFollower(new PathFollower());

    QObject::connect(camera.get(), SIGNAL(onNewLeftImage(ImageData)), barrelFinder.get(), SLOT(onNewImage(ImageData)));
    QObject::connect(camera.get(), SIGNAL(onNewLeftImage(ImageData)), lineDetector.get(), SLOT(onImageEvent(ImageData)));
    QObject::connect(lidar.get(), SIGNAL(onNewData(LidarState)), mapBuilder.get(), SLOT(onLidarData(LidarState)));
    QObject::connect(lineDetector.get(), SIGNAL(onNewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)), mapBuilder.get(), SLOT(onCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)));
    QObject::connect(barrelFinder.get(), SIGNAL(newCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXY)), mapBuilder.get(), SLOT(onCloudFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointXY)));

    QObject::connect(controller.get(), &Controller::onNewWaypoint, [=](GPSData waypoint){
        planner->OnNewGoalPos(posTracker->WaypointToPosition(waypoint));
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
    if(pos != modules.end())
        modules.erase(pos);
    modules.push_back(gps);
    auto trackerPos = find("PositionTracker");
    if(trackerPos != modules.end()) {
        shared_ptr<BasicPositionTracker> tracker = dynamic_pointer_cast<BasicPositionTracker>(*trackerPos);
        tracker->ChangeGPS(dynamic_pointer_cast<GPS>(gps));
    }
}

module_list_t::iterator CompetitionCoordinator::find(string name) {
    for(auto iter = modules.begin(); iter != modules.end(); iter++) {
        if((*iter)->moduleName() == name)
            return iter;
    }
    return modules.end();
}

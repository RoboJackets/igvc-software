#include "adapterfactory.h"
#include <adapters/joystickadapter.h>
#include <adapters/lidaradapter.h>
#include <adapters/cameraadapter.h>
#include <adapters/gpsadapter.h>
#include <adapters/imuadapter.h>
#include <adapters/lightshieldadapter.h>
#include <adapters/mapadapter.h>
#include <adapters/motorboardadapter.h>
#include <adapters/pathadapter.h>
#include <adapters/positiontrackeradapter.h>
#include <adapters/competitioncontrolleradapter.h>

QWidget* AdapterFactory::getAdapterForModule(std::shared_ptr<Module> module, QWidget *parent) {
    std::string name = module->moduleName();
    if(name == "Joystick")
        return new JoystickAdapter(dynamic_pointer_cast<Joystick>(module), parent);
    if(name == "LIDAR")
        return new LidarAdapter(dynamic_pointer_cast<Lidar>(module), parent);
    if(name == "IMU")
        return new IMUAdapter(dynamic_pointer_cast<IMU>(module), parent);
    if(name == "Camera")
        return new CameraAdapter(parent);
    if(name == "GPS")
        return new GPSAdapter(dynamic_pointer_cast<GPS>(module), parent);
    if(name == "MapBuilder")
        return new MapAdapter(dynamic_pointer_cast<MapBuilder>(module), parent);
    if(name == "PathPlanner")
        return new PathAdapter(dynamic_pointer_cast<PathPlanner>(module), parent);
    if(name == "PositionTracker")
        return new PositionTrackerAdapter(dynamic_pointer_cast<BasicPositionTracker>(module), parent);

    return new QWidget();
}

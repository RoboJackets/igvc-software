#include "lightcontroller.h"
#include <common/logger/logger.h>

LightController::LightController()
    : port("/dev/igvc_lights_arduino")
{
    port.setBaudRate(9600);
    if( !port.open(QIODevice::ReadWrite) )
        Logger::Log(LogLevel::Error, "Could not connect to lights Arduino.");
}

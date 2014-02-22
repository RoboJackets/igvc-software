#include "lms200.h"
#include <QString>
#include <common/logger/logger.h>

using namespace std;
using namespace SickToolbox;

LMS200::LMS200()
    : _device("/dev/ttyLidar")
{
    Logger::Log(LogLevel::Info, "Initializing SICK LMS200 device...");
    try {
        _device.Initialize(SickLMS::SICK_BAUD_9600);
    }
    catch(...) {
        Logger::Log(LogLevel::Error, "Failed to initialize SICK LMS200 device.");
        return;
    }
    try {
        // Tells the device to generate full 180 deg scans at 0.5 deg resolution.
        _device.SetSickVariant(SickLMS::SICK_SCAN_ANGLE_180, SickLMS::SICK_SCAN_RESOLUTION_50);
    }
    catch(...) {
        Logger::Log(LogLevel::Error, "Failed to set SICK LMS200 scanning variant.");
        return;
    }

    Logger::Log(LogLevel::Info, "SICK LMS200 device initialized.");

    _thread_running = true;
    _thread = boost::thread(boost::bind(&LMS200::thread_run, this));
}

void LMS200::thread_run()
{
    while(_thread_running)
    {
        unsigned int values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
        unsigned int num_values_returned = 0;
        try {
            _device.GetSickScan(values, num_values_returned);
        } catch(...) {
            Logger::Log(LogLevel::Error, "Failed to get scan data from SICK LMS200 device.");
            continue;
        }

        LidarState state;
        for(uint i = 0; i < num_values_returned; i++)
        {
            state.points[i] = LidarPoint();
            state.points[i].distance = values[i] / 1000.0; // mm to m conversion
            state.points[i].raw = values[i];
            state.points[i].angle = 0.00872664626 * i; // device defaults to 0.5 degree resolution (0.00872664626 radians)
            state.points[i].valid = state.points[i].distance <= 8.0;
        }
        for(uint i = num_values_returned; i < 1024; i++)
        {
            state.points[i] = LidarPoint();
            state.points[i].valid = false;
        }
        onNewData(state);
    }
}

LidarState LMS200::GetState()
{
    return LidarState();
}

LidarState LMS200::GetStateAtTime(timeval)
{
    return LidarState();
}

bool LMS200::IsWorking()
{
    return _device.IsInitialized() && _device.GetSickStatus() == SickLMS::SICK_STATUS_OK;
}

LMS200::~LMS200()
{
    Logger::Log(LogLevel::Info, "Uninitializing SICK LMS200 device...");
    _thread_running = false;
    _thread.join();
    try {
        _device.Uninitialize();
    }
    catch(...) {
        Logger::Log(LogLevel::Error, "Failed to uninitialize SICK LMS200 device.");
    }
    Logger::Log(LogLevel::Info, "SICK LMS200 device uninitialized.");
}

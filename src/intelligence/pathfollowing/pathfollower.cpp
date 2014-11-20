#include "pathfollower.h"
#include <time.h>
#include <common/config/configmanager.h>

PathFollower::PathFollower(QObject *)
{
    _moduleName = "PathFollower";
    _threadIsRunning = false;
}

bool PathFollower::isWorking()
{
    return true;
}

void PathFollower::onNewPath(path_t path)
{
    _path = path;
    _pathIndex = 0;
    if(!_threadIsRunning)
    {
        _threadIsRunning = true;
        _thread = boost::thread(boost::bind(&PathFollower::run, this));
    }
}

void PathFollower::run()
{
    while(_threadIsRunning)
    {
        if(_pathIndex < _path.size())
        {
            const SearchMove &move = _path[_pathIndex].first;
            double radius = move.V / move.W;
            double b = ConfigManager::Instance().getValue("Robot", "Baseline", 1.0);
            double left = ( radius - b/2.) * move.W;
            double right = ( radius + b/2.) * move.W;
            newMotorCommand(MotorCommand(left, right));
            time_t now;
            std::time(&now);
            if(now - _cmdStartTime > move.DeltaT)
            {
                std::time(&_cmdStartTime);
                _pathIndex++;

            }
        }
        else
        {
            newMotorCommand(MotorCommand(0,0));
        }
    }
    _threadIsRunning = false;
}

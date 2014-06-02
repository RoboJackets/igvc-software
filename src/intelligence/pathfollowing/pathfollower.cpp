#include "pathfollower.h"
#include <time.h>
#include <common/config/configmanager.h>

PathFollower::PathFollower(QObject *parent) :
    QObject(parent)
{
    _threadIsRunning = false;
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
    }
    _threadIsRunning = false;
}

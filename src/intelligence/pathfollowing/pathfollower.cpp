#include "pathfollower.h"
#include <time.h>

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
        time_t now;
        std::time(&now);
        if(now - _cmdStartTime > _path[_pathIndex].first.DeltaT)
        {
            std::time(&_cmdStartTime);
            _pathIndex++;

        }
    }
    _threadIsRunning = false;
}

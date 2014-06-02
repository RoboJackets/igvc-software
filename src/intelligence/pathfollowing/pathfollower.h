#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <QObject>
#include <common/datastructures/PathType.hpp>
#include <boost/thread.hpp>
#include <common/datastructures/MotorCommand.hpp>

class PathFollower : public QObject
{
    Q_OBJECT
public:
    explicit PathFollower(QObject *parent = 0);

signals:
    void newMotorCommand(MotorCommand cmd);

public slots:
    void onNewPath(path_t path);

protected:
    void run();

private:
    path_t _path;
    uint _pathIndex;
    time_t _cmdStartTime;

    boost::mutex _pathMutex;
    boost::thread _thread;
    bool _threadIsRunning;

};

#endif // PATHFOLLOWER_H

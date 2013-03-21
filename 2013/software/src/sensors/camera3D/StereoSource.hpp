#ifndef STEREOSOURCE_H
#define STEREOSOURCE_H

#include "sensors/camera3D/StereoPair.hpp"
#include "events/Event.hpp"
#include <boost/thread/thread.hpp>

class StereoSource
{
    public:
        StereoSource() : _running(true) {}
        inline bool Running() {return _running;}
        inline void Running(bool newState) {_running = newState;}
        inline void LockImages() {_imagesLock.lock();}
        inline void UnlockImages() {_imagesLock.unlock();}
        Event<StereoPair> onNewData;
        virtual ~StereoSource() {}
    private:
        bool _running;
        boost::mutex _imagesLock;
};

#endif // STEREOSOURCE_H

#ifndef JOYSTICK_H
#define JOYSTICB_H

#include <boost/thread.hpp>
#include <vector>
#include "common/events/Event.hpp"

using namespace std;

class JoystickState {
    public:
        vector<int> axes;
        vector<int> buttons;
};

class Joystick
{
    public:
        Joystick();
        bool isOpen();
        void disconnect();
        virtual ~Joystick();
        Event<JoystickState> onNewData;
    protected:
    private:
        boost::thread _iothread;
        int _joystick_fd;
        void threadRun();
        bool _isOpen;
        JoystickState _state;
};

#endif // JOYSTICK_H

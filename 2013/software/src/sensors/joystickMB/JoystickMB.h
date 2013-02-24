#ifndef JOYSTICKMB_H
#define JOYSTICKMB_H

#include <boost/thread.hpp>
#include <vector>
#include "events/Event.hpp"

using namespace std;

class JoystickState {
    public:
        vector<int> axes;
        vector<int> buttons;
};

class JoystickMB
{
    public:
        JoystickMB();
        bool isOpen();
        void disconnect();
        virtual ~JoystickMB();
        Event<JoystickState> onNewData;
    protected:
    private:
        boost::thread _iothread;
        int _joystick_fd;
        void threadRun();
        bool _isOpen;
        JoystickState _state;
};

#endif // JOYSTICKMB_H

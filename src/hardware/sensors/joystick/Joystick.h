#ifndef JOYSTICK_H
#define JOYSTICK_H

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
        int NumAxes();
        int NumButtons();
    protected:
    private:
        boost::thread _iothread;
        int _joystick_fd;
        void threadRun();
        bool _isOpen;
        JoystickState _state;
        int _numAxes;
        int _numButtons;
};

#endif // JOYSTICK_H

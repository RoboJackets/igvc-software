#include "Joystick.h"
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <linux/joystick.h>
#include "common/logger/logger.h"

Joystick::Joystick()
{

    qRegisterMetaType<JoystickState>("JoystickState");
    _joystick_fd = open("/dev/input/js0", O_RDONLY);
    if(_joystick_fd < 0)
    {
        Logger::Log(LogLevel::Warning, "Could not open joystick.");
        _isOpen = false;
    } else {
        _isOpen = true;
        Logger::Log(LogLevel::Info, "Joystick opened.");

        unsigned int numAxes=0, numButtons=0;
        ioctl(_joystick_fd, JSIOCGAXES, &numAxes);
        ioctl(_joystick_fd, JSIOCGBUTTONS, &numButtons);

        _numAxes = numAxes;
        _numButtons = numButtons;

        _state.axes.resize(numAxes);
        _state.buttons.resize(numButtons);

        _iothread = boost::thread(boost::bind(&Joystick::threadRun, this));
    }
}

void Joystick::threadRun()
{
    while(_isOpen)
    {
        struct pollfd pfd;
        pfd.fd = _joystick_fd;
        pfd.events = POLLIN | POLLHUP;

        int ret = poll(&pfd, 1, 0);

        if(ret == 0)
        {
            // no data
            continue;
        } else if(ret < 0) {
            // Poll error
            if(errno != EINTR)
            {
                std::stringstream msg;
                msg << "Joystick poll error: " << errno;
                Logger::Log(LogLevel::Warning, msg.str());
                continue;
            }
        }

        struct js_event event;
        if(read(_joystick_fd, &event, sizeof(event)) != sizeof(event))
        {
            continue;
        }

        if(event.type == JS_EVENT_BUTTON)
        {
            _state.buttons[event.number] = event.value;
        } else if(event.type == JS_EVENT_AXIS)
        {
            _state.axes[event.number] = event.value;
        }

        onNewData(_state);
    }
}

bool Joystick::isOpen()
{
    return _isOpen;
}

void Joystick::disconnect()
{
    if(_isOpen)
    {
        _isOpen = false;
        _iothread.join();
        close(_joystick_fd);
        _joystick_fd = -1;
        _state.axes.clear();
        _state.buttons.clear();
        Logger::Log(LogLevel::Info, "Joystick closed.");
    }
}

Joystick::~Joystick()
{
    disconnect();
}

int Joystick::NumAxes()
{
    return _numAxes;
}

int Joystick::NumButtons()
{
    return _numButtons;
}

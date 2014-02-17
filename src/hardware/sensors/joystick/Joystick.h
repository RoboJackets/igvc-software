#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <boost/thread.hpp>
#include <vector>
#include <QObject>

using namespace std;

class JoystickState {
    public:
        vector<int> axes;
        vector<int> buttons;
};

/*!
 * \brief For connecting to Joystick devices.
 *
 * \author Matthew Barulic
 * \headerfile Joystick.h <hardware/sensors/joystick/Joystick.h>
 */
class Joystick : public QObject
{
Q_OBJECT
public:
    Joystick();
    /*! \brief Returns true if the device was successfully opened. */
    bool isOpen();
    /*! \brief Closes the connection to the device. */
    void disconnect();

    virtual ~Joystick();

    /*! \brief Returns the number of axes on the connected device.
     *  \note Some devices may label physical buttons (such as d-pads) as axes.
     */
    int NumAxes();
    /*! \brief Returns the number of buttons on the connected device.
     *  \note Some devices may label physical buttons (such as d-pads) as axes.
     */
    int NumButtons();

signals:
    void onNewData(JoystickState);

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

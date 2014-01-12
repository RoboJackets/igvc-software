#ifndef JOYSTICKADAPTER_H
#define JOYSTICKADAPTER_H

#include <QWidget>
#include <boost/thread.hpp>
#include "hardware/sensors/joystick/Joystick.h"
#include "common/events/Event.hpp"

namespace Ui {
class JoystickAdapter;
}

/*!
 * \brief Widget for displaying joystick data
 * \author Matthew Barulic
 */
class JoystickAdapter : public QWidget
{
    Q_OBJECT
    
public:
    explicit JoystickAdapter(Joystick *joystick, QWidget *parent = 0);
    ~JoystickAdapter();

protected:
    void paintEvent(QPaintEvent *e);
    
private:
    Ui::JoystickAdapter *ui;

    Joystick *_joystick;

    JoystickState _state;

    boost::mutex _mutex;

    void OnJoystickData(JoystickState state);
    LISTENER(JoystickAdapter, OnJoystickData, JoystickState)
};

#endif // JOYSTICKADAPTER_H

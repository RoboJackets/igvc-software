#ifndef JOYSTICKADAPTER_H
#define JOYSTICKADAPTER_H

#include <QWidget>
#include <boost/thread.hpp>
#include "hardware/sensors/joystick/Joystick.h"

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

private slots:
    void onJoystickData(JoystickState state);
    
private:
    Ui::JoystickAdapter *ui;

    Joystick *_joystick;

    JoystickState _state;

    boost::mutex _mutex;
};

#endif // JOYSTICKADAPTER_H

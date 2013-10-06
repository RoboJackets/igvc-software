#ifndef CAMERAADAPTER_H
#define CAMERAADAPTER_H

#include <QWidget>
#include <QRect>
#include <boost/thread.hpp>
#include "hardware/sensors/joystick/Joystick.h"
#include "common/events/Event.hpp"


namespace Ui {
class CameraAdapter;
}

class CameraAdapter : public QWidget
{
    Q_OBJECT

public:
    explicit CameraAdapter(/*Camera *camera,*/ QWidget *parent = 0);
    ~CameraAdapter();

protected:
    void paintEvent(QPaintEvent *e);
   // void resizeEvent(QResizeEvent *e);

private:
    Ui::CameraAdapter *ui;

    QImage currentLeftFrame;
    QImage currentRightFrame;
    QImage currentDepthFrame;
    QImage currentPointCloudFrame;


    //Camera *camera;

    //CameraState _state;

    boost::mutex _mutex;

    void paint(QPainter* painter);

    void OnCameraData(/*CameraState state*/);

    /*void OnJoystickData(JoystickState state);
    LISTENER(JoystickAdapter, OnJoystickData, JoystickState)*/

};

#endif // CAMERAADAPTER_H

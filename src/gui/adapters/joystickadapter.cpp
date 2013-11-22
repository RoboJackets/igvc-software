#include "joystickadapter.h"
#include "ui_joystickadapter.h"
#include <QGridLayout>

JoystickAdapter::JoystickAdapter(Joystick *joystick, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoystickAdapter),
    LOnJoystickData(this)
{
    ui->setupUi(this);
    if(joystick)
    {
        _joystick = joystick;
        _joystick->onNewData += &LOnJoystickData;
    }
    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
}

JoystickAdapter::~JoystickAdapter()
{
    if(_joystick)
        _joystick->onNewData -= &LOnJoystickData;
    delete ui;
}

void JoystickAdapter::OnJoystickData(JoystickState state)
{
    if(isVisible())
    {
        _mutex.lock();
        _state = state;
        _mutex.unlock();
        update();
    }
}

void JoystickAdapter::paintEvent(QPaintEvent *e)
{
    _mutex.lock();
    ui->axesList->clear();
    for(auto i = 0u; i < _state.axes.size(); i++)
    {
        ui->axesList->addItem(tr("Axis %0 : %1").arg(i).arg(_state.axes[i]));
    }
    ui->buttonsList->clear();
    for(auto i = 0u; i < _state.buttons.size(); i++)
    {
        ui->buttonsList->addItem(tr("Button %0 : %1").arg(i).arg(_state.buttons[i]));
    }
    _mutex.unlock();
    QWidget::paintEvent(e);
}

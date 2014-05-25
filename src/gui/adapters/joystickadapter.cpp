#include "joystickadapter.h"
#include "ui_joystickadapter.h"
#include <QGridLayout>

JoystickAdapter::JoystickAdapter(std::shared_ptr<Joystick> joystick, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoystickAdapter)
{
    ui->setupUi(this);
    if(joystick.get())
    {
        _joystick = joystick;
        connect(_joystick.get(), SIGNAL(onNewData(JoystickState)), this, SLOT(onJoystickData(JoystickState)));
    }
    if(parent)
    {
        parent->setLayout(new QGridLayout());
    }
}

JoystickAdapter::~JoystickAdapter()
{
    if(_joystick.get() != nullptr)
    {
        disconnect(_joystick.get(), SIGNAL(onNewData(JoystickState)), this, SLOT(onJoystickData(JoystickState)));
    }
    delete ui;
}

void JoystickAdapter::onJoystickData(JoystickState state)
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

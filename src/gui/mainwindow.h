#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "batteryindicator.h"
#include "configtreemodel.h"
#include "mdiwindow.h"
#include <QMdiArea>
#include <QSignalMapper>
#include "hardware/sensors/joystick/Joystick.h"
#include "hardware/actuators/motors/MotorEncoderDriver2013.h"
#include "intelligence/JoystickDriver.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionFullscreen_triggered();

    void updateWindowMenu();
    void setActiveSubWindow(QWidget *window);

    void updateMenus();

    void openHardwareView(QModelIndex index);


    //TODO : We're gonna make this a real thing.
    //void setupHardwareStatusList();

    void on_joystickButton_toggled(bool checked);

    void on_playButton_clicked();

    void on_stopButton_clicked();


private:
    Ui::MainWindow *ui;
    ConfigTreeModel configTreeModel;
    QMdiArea* mdiArea;
    QSignalMapper *windowMapper;

    QIcon checkIcon;
    QIcon xIcon;

    JoystickDriver *_joystickDriver;
    Joystick *_joystick;

    MotorDriver *_motorController;

    bool isRunning, isPaused;

    MDIWindow *activeMdiChild();
    void setupMenus();
    MDIWindow* findWindowWithTitle(QString title);
};

#endif // MAINWINDOW_H

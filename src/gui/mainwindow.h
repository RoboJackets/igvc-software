#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "batteryindicator.h"
#include "configtreemodel.h"
#include "mdiwindow.h"
#include <QMdiArea>
#include "hardware/sensors/joystick/Joystick.h"
#include "hardware/actuators/motors/MotorEncoderDriver2013.h"
#include "intelligence/JoystickDriver.hpp"
#include <hardware/actuators/lights/lightcontroller.h>
#include <QLabel>
#include <QTimer>
#include <intelligence/coordinators/coordinator.hpp>

namespace Ui {
class MainWindow;
}

/*!
 * \brief The main window of the control application.
 * \author Matthew Barulic
 */
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

    void openHardwareView(QString label);

    void on_joystickButton_toggled(bool checked);

    void on_playButton_clicked();

    void on_stopButton_clicked();

    void on_actionStatus_Bar_toggled(bool arg1);

    void on_saveConfigButton_clicked();

    void on_loadConfigButton_clicked();

    void on_actionHemisphere_A100_triggered();

    void on_actionSimulatedGPS_triggered();

    void on_actionClearLogs_triggered();

    void on_actionOutback_A321_triggered();

    void on_actionSimulatedLidar_triggered();

    void on_actionLMS_200_triggered();

    void on_actionLoad_Waypoint_File_triggered();

    void updateTimer();

protected:
    void closeEvent(QCloseEvent *);

private:
    Ui::MainWindow *ui;
    ConfigTreeModel configTreeModel;
    QMdiArea* mdiArea;
    QSignalMapper *windowMapper;

    QIcon checkIcon;
    QIcon xIcon;

    std::shared_ptr<JoystickDriver> _joystickDriver;
    std::shared_ptr<Joystick> _joystick;

    std::shared_ptr<MotorDriver> _motorController;

    std::shared_ptr<LightController> _lights;

    std::unique_ptr<Coordinator> _coordinator;

    bool isRunning, isPaused;
    int curTime;
    QLabel* timeLabel;
    QTimer* qtimer;

    MDIWindow *activeMdiChild();
    void setupMenus();
    MDIWindow* findWindowWithTitle(QString title);
    void updateHardwareStatusList();

    void restartAdapter(QString title);
};

#endif // MAINWINDOW_H

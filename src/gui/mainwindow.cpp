#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "adapters/adapterfactory.h"
#include "adapters/joystickadapter.h"
#include "adapters/motorboardadapter.h"
#include "adapters/competitioncontrolleradapter.h"
#include "adapters/logvieweradapter.h"
#include "adapters/lightshieldadapter.h"

#include <hardware/sensors/gps/simulatedgps.h>
#include <hardware/sensors/gps/nmeacompatiblegps.h>
#include <hardware/sensors/camera/StereoPlayback.h>
#include <hardware/sensors/camera/StereoImageRepeater.h>
#include <hardware/sensors/camera/Bumblebee2.h>
#include <hardware/sensors/IMU/Ardupilot.h>
#include <hardware/sensors/lidar/SimulatedLidar.h>
#include <hardware/sensors/lidar/lms200.h>

#include <QMdiSubWindow>
#include <QTextEdit>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>

#include <iostream>

#include <intelligence/coordinators/competitioncoordinator.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    configTreeModel.populateModel();
    ui->configTree->setModel(configTreeModel.model());
    mdiArea = ui->mainDisplayArea;
    connect(mdiArea, SIGNAL(subWindowActivated(QMdiSubWindow*)),
            this, SLOT(updateMenus()));
    windowMapper = new QSignalMapper(this);
    connect(windowMapper, SIGNAL(mapped(QWidget*)),
            this, SLOT(setActiveSubWindow(QWidget*)));

    checkIcon = QIcon(QString(":/images/Checkmark"));
    xIcon = QIcon(QString(":/images/Close"));

    connect(ui->hardwareStatusList, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(openHardwareView(QModelIndex)));

    Logger::setSatusBar(ui->statusBar);

    setupMenus();
    updateWindowMenu();

    _motorController = std::shared_ptr<MotorDriver>(new MotorEncoderDriver2013);
    ui->hardwareStatusList->addItem("Motor Board");

    _joystick = std::shared_ptr<Joystick>(new Joystick);
    ui->joystickButton->setEnabled(_joystick->isOpen());
    ui->hardwareStatusList->addItem("Joystick");

    _lights = std::shared_ptr<LightController>(new LightController());
    connect(_lights.get(), SIGNAL(onBatteryLevelChanged(int)), ui->batteryIndicator, SLOT(onBatteryLevelChanged(int)));
    connect(_lights.get(), SIGNAL(onEStopStatusChanged(bool)), ui->statusImage, SLOT(onEStopStatusChanged(bool)));
    connect(_lights.get(), SIGNAL(onEStopStatusChanged(bool)), _motorController.get(), SLOT(onEStopStatusChanged(bool)));
    ui->hardwareStatusList->addItem("Light Controller");

    _coordinator = unique_ptr<Coordinator>(new CompetitionCoordinator());

    ui->actionLMS_200->setChecked(true);

    ui->actionOutback_A321->setChecked(true);

    _joystickDriver = std::shared_ptr<JoystickDriver>(new JoystickDriver(_joystick));

    connect(_coordinator.get(), SIGNAL(finished()), this, SLOT(on_stopButton_clicked()));

    updateHardwareStatusList();

    isRunning = false;
    isPaused = false;
    ui->stopButton->setVisible(false);

    timeLabel = new QLabel("Time: 0:00:00");
    ui->statusBar->addPermanentWidget(timeLabel);
    qtimer = new QTimer(this);
    connect(qtimer, SIGNAL(timeout()), this, SLOT(updateTimer()));
    qtimer->start(1000);
    curTime = 0;
}

void MainWindow::setupMenus()
{
    connect(ui->actionClose, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->menuWindow, SIGNAL(aboutToShow()), this, SLOT(updateWindowMenu()));
    connect(ui->actionClose_2, SIGNAL(triggered()), mdiArea, SLOT(closeActiveSubWindow()));
    connect(ui->actionClose_All, SIGNAL(triggered()), mdiArea, SLOT(closeAllSubWindows()));
    connect(ui->actionCascade, SIGNAL(triggered()), mdiArea, SLOT(cascadeSubWindows()));
    connect(ui->actionTile, SIGNAL(triggered()), mdiArea, SLOT(tileSubWindows()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openHardwareView(QModelIndex index)
{
    QString labelText = ui->hardwareStatusList->item(index.row())->text();
    if(MDIWindow* window = findWindowWithTitle(labelText))
    {
        if(!window->isVisible())
            window->show();
    }
    else
    {
        using namespace std;
        MDIWindow *newWindow = new MDIWindow;
        newWindow->setWindowTitle(labelText);
        newWindow->setLayout(new QGridLayout);

        QWidget* adapter = nullptr;

        if(labelText == "Motor Board")
        {
            adapter = new MotorBoardAdapter(_motorController);
        }
        else if(labelText == "Joystick")
        {
            adapter = new JoystickAdapter(_joystick);
        } else if(labelText == "Light Controller") {
            adapter = new LightShieldAdapter(_lights);
        } else {
            shared_ptr<Module> module = _coordinator->getModuleWithName(labelText.toStdString());
            if(module.get())
                adapter = AdapterFactory::getAdapterForModule(module);
        }

        if(adapter != nullptr) {
            newWindow->layout()->addWidget(adapter);

            mdiArea->addSubWindow(newWindow);
            newWindow->show();
        }
    }

    updateWindowMenu();
}

void MainWindow::on_actionFullscreen_triggered()
{
    if(ui->actionFullscreen->isChecked())
        this->showFullScreen();
    else
        this->showNormal();
}

void MainWindow::updateMenus()
{
    bool hasMdiChild = (activeMdiChild() != 0);

    ui->actionTile->setEnabled(hasMdiChild);
    ui->actionCascade->setEnabled(hasMdiChild);
    ui->actionClose_2->setEnabled(hasMdiChild);
    ui->actionClose_All->setEnabled(hasMdiChild);
}

void MainWindow::updateWindowMenu()
{
    for(int i = ui->menuWindow->actions().size()-1; i > 5; i--)
    {
        ui->menuWindow->removeAction(ui->menuWindow->actions().at(i));
    }

    QList<QMdiSubWindow *> windows = mdiArea->subWindowList();

    for (int i = 0; i < windows.size(); ++i) {
        MDIWindow *child = qobject_cast<MDIWindow *>(windows.at(i)->widget());

        QString text = windows.at(i)->windowTitle();
        QAction *action  = ui->menuWindow->addAction(text);
        action->setCheckable(true);
        action ->setChecked(child == activeMdiChild());
        connect(action, SIGNAL(triggered()), windowMapper, SLOT(map()));
        windowMapper->setMapping(action, windows.at(i));
    }
}

MDIWindow* MainWindow::activeMdiChild()
{
    if (QMdiSubWindow *activeSubWindow = mdiArea->activeSubWindow())
        return qobject_cast<MDIWindow *>(activeSubWindow->widget());
    return 0;
}

void MainWindow::setActiveSubWindow(QWidget *window)
{
    if (!window)
        return;
    mdiArea->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(window));
}

MDIWindow* MainWindow::findWindowWithTitle(QString title)
{
    foreach(QMdiSubWindow *window, mdiArea->subWindowList())
    {
        MDIWindow *mdiChild = qobject_cast<MDIWindow*>(window->widget());
        if(mdiChild && mdiChild->windowTitle() == title)
            return mdiChild;
    }
    return nullptr;
}

void MainWindow::on_joystickButton_toggled(bool checked)
{
    ui->joystickButton->setStyleSheet(tr("background-color: %1").arg(checked ? "rgb(0,255,0)" : "default"));
    if(checked)
    {
        if(isRunning)
            on_stopButton_clicked();
        connect(_joystickDriver.get(), SIGNAL(onNewMotorCommand(MotorCommand)), _motorController.get(), SLOT(setMotorCommand(MotorCommand)));
    }
    else
    {
        disconnect(_joystickDriver.get(), SIGNAL(onNewMotorCommand(MotorCommand)), _motorController.get(), SLOT(setMotorCommand(MotorCommand)));
    }
}

void MainWindow::on_playButton_clicked()
{
    if(isRunning)
    {
        if(isPaused)
        {
            ui->playButton->setIcon(QIcon(":/images/Pause"));
        }
        else
        {
            ui->playButton->setIcon(QIcon(":/images/Play"));
        }
        isPaused = !isPaused;
        disconnect(_coordinator.get(), SIGNAL(newMotorCommand(MotorCommand)), _motorController.get(), SLOT(setMotorCommand(MotorCommand)));
        _motorController->setMotorCommand(MotorCommand(0.,0.));
    }
    else
    {
        ui->joystickButton->setChecked(false);
        ui->playButton->setIcon(QIcon(":/images/Pause"));
        ui->stopButton->setVisible(true);
        isRunning = true;
        connect(_coordinator.get(), SIGNAL(newMotorCommand(MotorCommand)), _motorController.get(), SLOT(setMotorCommand(MotorCommand)));
        _motorController->setMotorCommand(MotorCommand(1.,1.));
    }
    _lights->setSafetyLight(isRunning);
}

void MainWindow::on_stopButton_clicked()
{
    if(isRunning)
    {
        ui->playButton->setIcon(QIcon(":/images/Play"));
        ui->stopButton->setVisible(false);
        isRunning = false;
        isPaused = false;
        disconnect(_coordinator.get(), SIGNAL(newMotorCommand(MotorCommand)), _motorController.get(), SLOT(setMotorCommand(MotorCommand)));
        _motorController->setMotorCommand(MotorCommand(0.,0.));
    }
    _lights->setSafetyLight(isRunning);
    curTime = 0;
    timeLabel->setText("Time: 0:00:00");
}

void MainWindow::on_actionStatus_Bar_toggled(bool checked)
{
    if(checked)
    {
        ui->statusBar->show();
        Logger::setSatusBar(ui->statusBar);
    }
    else
    {
        ui->statusBar->hide();
        Logger::setSatusBar(0);
    }
}

void MainWindow::on_saveConfigButton_clicked()
{
    ConfigManager::Instance().save();
}

void MainWindow::on_loadConfigButton_clicked()
{
    if(ConfigManager::Instance().hasUnsavedChanges() && QMessageBox::warning(this, "Unsaved Changes in Configuration", "There are unsaved changes in your config file. Would you like to save them now?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
        ConfigManager::Instance().save();
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Configuration File"), "", tr("XML Files(*.xml)"));
    if(!fileName.isEmpty())
    {
        ConfigManager::Instance().load(fileName.toStdString());
    }
}

void MainWindow::on_actionHemisphere_A100_triggered()
{
    ui->actionSimulatedGPS->setChecked(false);
    ui->actionHemisphere_A100->setChecked(true);
    ui->actionOutback_A321->setChecked(false);
    _coordinator->changeGPS(std::shared_ptr<GPS>(new NMEACompatibleGPS("/dev/ttyGPS", 4800)));
    updateHardwareStatusList();
}

void MainWindow::on_actionSimulatedGPS_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Simulated GPS Data File"), "", tr("Text Files(*.txt)"));
    if(fileName.length() > 0)
    {
        ui->actionSimulatedGPS->setChecked(true);
        ui->actionHemisphere_A100->setChecked(false);
        ui->actionOutback_A321->setChecked(false);
        _coordinator->changeGPS(std::shared_ptr<GPS>(new SimulatedGPS(fileName.toStdString())));
//        MDIWindow *window = findWindowWithTitle("GPS");
//        if( window != nullptr)
//        {
//            QWidget* p = (QWidget*)window->parent();
//            if(p != nullptr)
//                p->close();
//        }
        updateHardwareStatusList();
    }
}

void MainWindow::updateHardwareStatusList()
{
    ui->hardwareStatusList->clear();
    ui->hardwareStatusList->addItems(QStringList({"Joystick","Motor Board","Light Controller"}));
    ui->hardwareStatusList->findItems("Joystick", Qt::MatchExactly).at(0)->setIcon(_joystick->isWorking() ? checkIcon : xIcon);
    ui->hardwareStatusList->findItems("Motor Board", Qt::MatchExactly).at(0)->setIcon(_motorController->isWorking() ? checkIcon : xIcon);
    ui->hardwareStatusList->findItems("Light Controller", Qt::MatchExactly).at(0)->setIcon(_lights->isConnected() ? checkIcon : xIcon);
    for(shared_ptr<Module> module : _coordinator->getModules()) {
        ui->hardwareStatusList->addItem(module->moduleName().c_str());
        ui->hardwareStatusList->findItems(module->moduleName().c_str(), Qt::MatchExactly).at(0)->setIcon(module->isWorking() ? checkIcon : xIcon);
    }
}

void MainWindow::on_actionClearLogs_triggered()
{
    Logger::Clear();
}

void MainWindow::on_actionLogViewer_triggered()
{
    MDIWindow *newWindow = new MDIWindow;
    newWindow->setWindowTitle("Log Viewer");
    newWindow->setLayout(new QGridLayout);

    QWidget* adapter;
    adapter = new LogViewerAdapter();

    newWindow->layout()->addWidget(adapter);

    mdiArea->addSubWindow(newWindow);
    newWindow->show();
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    mdiArea->closeAllSubWindows();
    ConfigManager::Instance().save();
    QMainWindow::closeEvent(e);
}

void MainWindow::on_actionOutback_A321_triggered()
{
    ui->actionSimulatedGPS->setChecked(false);
    ui->actionHemisphere_A100->setChecked(false);
    ui->actionOutback_A321->setChecked(true);
    _coordinator->changeGPS(std::shared_ptr<GPS>(new NMEACompatibleGPS("/dev/ttyGPS", 19200)));
    updateHardwareStatusList();
}

void MainWindow::on_actionSimulatedLidar_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Simulated Lidar Data File"), "", tr("CSV Files(*.csv)"));
    if(!fileName.isEmpty())
    {
        ui->actionLMS_200->setChecked(false);
        ui->actionSimulatedLidar->setChecked(true);
        auto newDevice = std::shared_ptr<Lidar>(new SimulatedLidar);
        ((SimulatedLidar*)newDevice.get())->loadFile(fileName.toStdString().c_str());
        _coordinator->changeLidar(newDevice);
        updateHardwareStatusList();
    }
}

void MainWindow::on_actionLMS_200_triggered()
{
    ui->actionLMS_200->setChecked(true);
    ui->actionSimulatedLidar->setChecked(false);
    _coordinator->changeLidar(std::shared_ptr<Lidar>(new LMS200));
    updateHardwareStatusList();
}

void MainWindow::on_actionLoad_Waypoint_File_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Waypoint Data File"), "", tr("CSV Files(*.csv)"));
//    if(!fileName.isEmpty())
//        _waypointSource->openFile(fileName.toStdString());
    cerr << "NOT IMPLEMENTED YET" << endl;
}

void MainWindow::updateTimer()
{
    if(isRunning && !isPaused)
    {
        curTime++;
        int hour = curTime/3600;
        int second = curTime % 3600;
        int minute = second/60;
        second = second % 60;

        QString str;
        str.sprintf("Time: %d:%02d:%02d", hour, minute, second);
        timeLabel->setText(str);
    }
}

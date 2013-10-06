#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMdiSubWindow>
#include <QTextEdit>
#include "adapters/joystickadapter.h"
#include "adapters/cameraadapter.h"

#include <iostream>

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

    setupHardwareStatusList();

    connect(ui->hardwareStatusList, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(openHardwareView(QModelIndex)));

    setupMenus();
    updateWindowMenu();

    _joystick = new Joystick;
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
    delete _joystick;
    delete ui;
}

void MainWindow::setupHardwareStatusList()
{
    //Adds all of the clickable hardware elements to the sidebar

    ui->hardwareStatusList->addItem("Joystick");
    ui->hardwareStatusList->addItem("Camera");
}

void MainWindow::openHardwareView(QModelIndex index)
{
    //getting the name of the element
    string hardwareID = index.data().toString().toUtf8().constData();

    if(hardwareID=="Joystick")
    {
        if(MDIWindow* window = findWindowWithTitle("Joystick"))
        {
            if(!window->isVisible())
                window->show();
        }
        else
        {
            using namespace std;
            MDIWindow *newWindow = new MDIWindow;
            newWindow->setWindowTitle("Joystick");
            JoystickAdapter *adapter = new JoystickAdapter(_joystick);
            newWindow->setLayout(new QGridLayout);
            newWindow->layout()->addWidget(adapter);
            mdiArea->addSubWindow(newWindow);
            newWindow->show();
        }
    }
    if(hardwareID=="Camera")
    {
        if(MDIWindow* window = findWindowWithTitle("Camera"))
        {
            if(!window->isVisible())
                window->show();
        }
        else
        {
            using namespace std;
            MDIWindow *newWindow = new MDIWindow;
            newWindow->setWindowTitle("Camera");
            CameraAdapter *adapter = new CameraAdapter();
            newWindow->setLayout(new QGridLayout);
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
    return 0;
}

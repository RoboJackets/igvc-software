#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMdiSubWindow>
#include <QTextEdit>
#include "adapters/joystickadapter.h"

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

    ui->hardwareStatusList->addItem("Joystick");
    connect(ui->hardwareStatusList, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(openHardwareView(QModelIndex)));

    setupMenus();
    updateWindowMenu();

    _joystick = new Joystick;
    _joystickDriver = new JoystickDriver(&_joystick->onNewData);
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

        //JoystickAdapter *adapter = new JoystickAdapter(_joystick);
        QWidget* adapter;

        if(labelText == "Joystick")
        {
            adapter = new JoystickAdapter(_joystick);
        }
        else
        {
            adapter = new QWidget();
        }

        newWindow->layout()->addWidget(adapter);

        mdiArea->addSubWindow(newWindow);
        newWindow->show();
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

void MainWindow::on_joystickButton_toggled(bool checked)
{
    if(checked)
    {

    }
    else
    {

    }
}

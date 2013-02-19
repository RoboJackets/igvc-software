#include "mainwindow.h"
#include "qpainter.h"
#include "ui_mainwindow.h"
#include "qfiledialog.h"

using namespace IGVC::Sensors;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    removeToolBar(ui->toolBar);
    ui->lidarView->setScale(0.25);
    ui->horizontalSlider->setRange(10, 10000);
    ui->horizontalSlider->setValue(ui->horizontalSlider->maximum()/4);
    switchMode(Default);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    double scale = value / 10.00;
    ui->lidarView->setScale(scale);
}

void MainWindow::closeEvent(QCloseEvent *)
{
    if(_mode == Nav200)
        _nav200->stop();
}

void MainWindow::on_actionCapture_triggered()
{
    ui->lidarView->capture();
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_actionNAV200_triggered()
{
    switchMode(Nav200);
}

void MainWindow::on_actionLoad_File_triggered()
{
    _filePath = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                     "",
                                                     tr("Files (*.csv)"));
    switchMode(File);
}

void MainWindow::on_actionDefault_triggered()
{
    switchMode(Default);
}

void MainWindow::switchMode(Mode newMode)
{
    NAV200 *_newNav;
    SimulatedLidar *_newSim;

    // Make new lidar
    switch(newMode)
    {
    case Nav200:
        _newNav = new NAV200();
        ui->lidarView->setLidar(_newNav);
        break;
    case File:
        _newSim = new SimulatedLidar();
        _newSim->loadFile(_filePath.toStdString().c_str());
        ui->lidarView->setLidar(_newSim);
        break;
    case Default:
        _newSim = new SimulatedLidar();
        ui->lidarView->setLidar(_newSim);
        break;
    case None:
        // Do nothing
        break;
    }

    // Clean up old lidar
    switch(_mode)
    {
    case Nav200:
        _nav200->stop();
        delete _nav200;
        break;
    case File:
        delete _simulated;
        break;
    case Default:
        delete _simulated;
        break;
    case None:
        // Do nothing
        break;
    }

    // Set local pointers
    switch(newMode)
    {
    case Nav200:
        _nav200 = _newNav;
        break;
    case File:
        _simulated = _newSim;
        break;
    case Default:
        _simulated = _newSim;
        break;
    }

    // Switch mode
    _mode = newMode;
}

void MainWindow::on_actionLines_triggered()
{
    ui->lidarView->setViewMode(LidarDisplayWidget::Lines);
}

void MainWindow::on_actionPoints_triggered()
{
    ui->lidarView->setViewMode(LidarDisplayWidget::Points);
}

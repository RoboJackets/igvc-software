#include "mainwindow.h"
#include <QPainter>
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QTextEdit>
#include <fstream>

using namespace IGVC::Sensors;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _lidar = 0;
    ui->setupUi(this);
    removeToolBar(ui->toolBar);
    ui->lidarView->setScale(0.25);
    ui->horizontalSlider->setRange(10, 10000);
    ui->horizontalSlider->setValue(ui->horizontalSlider->maximum()/4);
    ui->actionLines->setEnabled(false);
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
    if(_lidar)
        delete _lidar;
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
    if(newMode == _mode && newMode != File)
        return;

    ui->lidarView->clearDeviceListeners();

    NAV200 *newNav=0;
    SimulatedLidar *newSim=0;

    // Make new lidar
    switch(newMode)
    {
    case Nav200:
        newNav = new NAV200();
        ui->lidarView->setLidar(newNav);
        ui->lidarView->shouldUpdateOnScaling = false;
        break;
    case File:
        newSim = new SimulatedLidar();
        newSim->loadFile(_filePath.toStdString().c_str());
        ui->lidarView->setLidar(newSim);
        break;
    case Default:
        newSim = new SimulatedLidar();
        ui->lidarView->setLidar(newSim);
        break;
    default:
        newSim = new SimulatedLidar();
        ui->lidarView->setLidar(newSim);
        break;
    }

    // Clean up old lidar
    if(_mode==Nav200)
        ui->lidarView->shouldUpdateOnScaling = true;
    if(_lidar)
    {
        delete _lidar;
        _lidar = 0;
    }

    //Set local pointer
    if(newMode == Nav200)
    {
        _lidar = newNav;
    } else if(newMode == File || newMode == Default)
    {
        _lidar = newSim;
    }

    // Switch mode
    _mode = newMode;
}

void MainWindow::on_actionLines_triggered()
{
    ui->lidarView->setViewMode(LidarDisplayWidget::Lines);
    ui->actionLines->setEnabled(false);
    ui->actionPoints->setEnabled(true);
}

void MainWindow::on_actionPoints_triggered()
{
    ui->lidarView->setViewMode(LidarDisplayWidget::Points);
    ui->actionLines->setEnabled(true);
    ui->actionPoints->setEnabled(false);
}

void MainWindow::on_actionAbout_triggered()
{
    std::fstream ifs("HelpText.html");
    if(ifs)
    {
        std::stringstream content;
        content << ifs.rdbuf();
        QTextEdit *help = new QTextEdit(this);
        help->setWindowFlags(Qt::Tool);
        help->setWindowTitle("About");
        help->setReadOnly(true);
        help->setHtml(content.str().c_str());
//        help->append(content.str().c_str());
        help->show();
    } else {
        std::cerr << "Error: Could not load help file." << std::endl;
    }
}

void MainWindow::on_actionInvalid_Points_triggered(bool checked)
{
    ui->lidarView->showInvalid = checked;
}

void MainWindow::on_actionFullscreen_triggered(bool checked)
{
    if(checked)
    {
        this->showFullScreen();
    } else {
        this->showNormal();
    }
}

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <signal.h>
#include <QDir>
#include <algorithm>

using namespace std;

MainWindow::MainWindow(int argc, char **argv) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    node(argc, argv)
{
    ui->setupUi(this);

    connect(&node, SIGNAL(newVelocityData(float)), ui->speedometer, SLOT(setValue(float)));
    connect(&node, &QNode::newNodesList, this, [=](QStringList nodes){
        ui->nodesListWidget->clear();
        for(auto node : nodes)
            ui->nodesListWidget->addItem(node);
    });
    connect(&node, SIGNAL(newBatteryLevel(int)), ui->batteryBar, SLOT(setValue(int)));
    connect(&node, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(&node, SIGNAL(newRosoutMessage(QString)), ui->statusbar, SLOT(showMessage(QString)));
    connect(&node, SIGNAL(newMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)), ui->mapWidget, SLOT(setMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)));
    connect(&node, &QNode::newEstopStatus, [=](bool enabled){
        if(enabled)
            ui->estopLabel->setStyleSheet("QLabel { background-color : rgb(0,255,0); }");
        else
            ui->estopLabel->setStyleSheet("QLabel { background-color : rgb(255,0,0); }");
    });

    if(node.init())
        cout << "ROS node initialized." << endl;
    else
        cerr << "ROS node failed to initialize." << endl;

    ui->batteryBar->setMaximum(255);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showEvent(QShowEvent *e)
{
    QList<int> sizes = {250, ui->splitter->width() - 250};
    ui->splitter->setSizes(sizes);
}

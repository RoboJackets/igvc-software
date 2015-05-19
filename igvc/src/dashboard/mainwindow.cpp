#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    node(argc, argv)
{
    ui->setupUi(this);

    connect(&node, SIGNAL(newVelocityData(float)), ui->speedometer, SLOT(setValue(float)));
    connect(&node, SIGNAL(newNodesList(QStringList)), this, SLOT(onNewNodesList(QStringList)));
    connect(&node, SIGNAL(newBatteryLevel(int)), ui->batteryBar, SLOT(setValue(int)));
    connect(&node, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(&node, SIGNAL(newRosoutMessage(QString)), ui->statusbar, SLOT(showMessage(QString)));
    connect(&node, SIGNAL(newMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)), ui->mapWidget, SLOT(setMap(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)));

    if(node.init())
        std::cout << "ROS node initialized." << std::endl;
    else
        std::cerr << "ROS node failed to initialize." << std::endl;

    ui->batteryBar->setMaximum(255);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onNewNodesList(QStringList nodes)
{
    ui->nodesListWidget->clear();
    for(auto node : nodes)
        ui->nodesListWidget->addItem(node);
}

void MainWindow::showEvent(QShowEvent *e)
{
    QList<int> sizes = {250, ui->splitter->width() - 250};
    ui->splitter->setSizes(sizes);
}

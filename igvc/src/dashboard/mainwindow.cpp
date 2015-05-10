#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    node(argc, argv)
{
    ui->setupUi(this);

    connect(&node, SIGNAL(newVelocityData(float)), this, SLOT(onNewVelocity(float)));
    connect(&node, SIGNAL(newNodesList(QStringList)), this, SLOT(onNewNodesList(QStringList)));

    if(node.init())
        std::cout << "ROS node initialized." << std::endl;
    else
        std::cerr << "ROS node failed to initialize." << std::endl;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onNewVelocity(float velocity)
{
    ui->widget->setValue(velocity);
}

void MainWindow::onNewNodesList(QStringList nodes)
{
    ui->listWidget->clear();
    for(auto node : nodes)
        ui->listWidget->addItem(node);
}

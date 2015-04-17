#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char **argv) :
    QMainWindow(0),
    ui(new Ui::MainWindow),
    node(argc, argv)
{
    ui->setupUi(this);

    connect(&node, SIGNAL(newVelocityData(float)), this, SLOT(onNewVelocity(float)));

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
    ui->widget->value = velocity;
    ui->widget->update();
}

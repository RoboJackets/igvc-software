#include "mainwindow.h"
#include "qpainter.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    removeToolBar(ui->toolBar);
    ui->lidarView->setScale(0.25);
}

MainWindow::~MainWindow()
{
    delete ui;
}

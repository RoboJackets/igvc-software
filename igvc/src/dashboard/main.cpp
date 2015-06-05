#include <QtGui>
#include <QApplication>
#include "mainwindow.h"
#include <pcl/point_cloud.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>("pcl::PointCloud<pcl::PointXYZ>::ConstPtr");

    MainWindow w(argc, argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    return app.exec();
}

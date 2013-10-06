#include <QApplication>
#include "mainwindow.h"

#include "common/config/configmanager.h"
#include "common/logger/logger.h"
#include <iostream>

int main(int argc, char *argv[])
{
    if(!ConfigManager::Instance().load())
        ConfigManager::Instance().save();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

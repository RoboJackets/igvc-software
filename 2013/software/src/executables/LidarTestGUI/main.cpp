#include <QtGui/QApplication>
#include "mainwindow.h"
#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

int main(int argc, char *argv[])
{
#ifdef Q_WS_X11
    XInitThreads();
#endif
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    return a.exec();
}

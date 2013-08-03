#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "batteryindicator.h"
#include "configtreemodel.h"
#include "mdiwindow.h"
#include <QMdiArea>
#include <QSignalMapper>
#include "hardware/sensors/joystick/Joystick.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionFullscreen_triggered();

    void updateWindowMenu();
    void setActiveSubWindow(QWidget *window);

    void updateMenus();

    void openHardwareView(QModelIndex index);

private:
    Ui::MainWindow *ui;
    ConfigTreeModel configTreeModel;
    QMdiArea* mdiArea;
    QSignalMapper *windowMapper;


    Joystick* _joystick;

    MDIWindow *activeMdiChild();
    void setupMenus();
    MDIWindow* findWindowWithTitle(QString title);
};

#endif // MAINWINDOW_H

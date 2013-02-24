#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "sensors/lidar/NAV200.h"
#include "sensors/lidar/SimulatedLidar.h"

using namespace IGVC::Sensors;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *);
    
private slots:
    void on_horizontalSlider_valueChanged(int value);

    void on_actionCapture_triggered();

    void on_actionExit_triggered();

    void on_actionNAV200_triggered();

    void on_actionLoad_File_triggered();

    void on_actionDefault_triggered();

    void on_actionLines_triggered();

    void on_actionPoints_triggered();

    void on_actionAbout_triggered();

private:
    Ui::MainWindow *ui;
    NAV200 *_nav200;
    SimulatedLidar *_simulated;
    QString _filePath;
    enum Mode {
        None,
        File,
        Default,
        Nav200
    };
    Mode _mode;
    void switchMode(Mode newMode);
};

#endif // MAINWINDOW_H

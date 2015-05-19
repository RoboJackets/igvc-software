#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qnode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv);
    ~MainWindow();

public slots:
    void onNewNodesList(QStringList nodes);

protected:
    void showEvent(QShowEvent *e) override;

private:
    Ui::MainWindow *ui;

    QNode node;
};

#endif // MAINWINDOW_H

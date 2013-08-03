#include "mdiwindow.h"
#include <QGridLayout>
#include <QCloseEvent>

#include <iostream>

MDIWindow::MDIWindow(QWidget *parent) :
    QWidget(parent)
{
    this->setMinimumSize(200,200);
}

/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sun Feb 17 04:43:20 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSlider>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "lidardisplaywidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionCapture;
    QAction *actionExit;
    QAction *actionNAV200;
    QAction *actionLoad_File;
    QAction *actionDefault;
    QAction *actionLines;
    QAction *actionPoints;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QSlider *horizontalSlider;
    LidarDisplayWidget *lidarView;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuLidar;
    QMenu *menuSimulated;
    QMenu *menuView;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(400, 300);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        actionCapture = new QAction(MainWindow);
        actionCapture->setObjectName(QString::fromUtf8("actionCapture"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionNAV200 = new QAction(MainWindow);
        actionNAV200->setObjectName(QString::fromUtf8("actionNAV200"));
        actionLoad_File = new QAction(MainWindow);
        actionLoad_File->setObjectName(QString::fromUtf8("actionLoad_File"));
        actionDefault = new QAction(MainWindow);
        actionDefault->setObjectName(QString::fromUtf8("actionDefault"));
        actionLines = new QAction(MainWindow);
        actionLines->setObjectName(QString::fromUtf8("actionLines"));
        actionPoints = new QAction(MainWindow);
        actionPoints->setObjectName(QString::fromUtf8("actionPoints"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalSlider = new QSlider(centralWidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(horizontalSlider);

        lidarView = new LidarDisplayWidget(centralWidget);
        lidarView->setObjectName(QString::fromUtf8("lidarView"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lidarView->sizePolicy().hasHeightForWidth());
        lidarView->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(lidarView);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuLidar = new QMenu(menuBar);
        menuLidar->setObjectName(QString::fromUtf8("menuLidar"));
        menuSimulated = new QMenu(menuLidar);
        menuSimulated->setObjectName(QString::fromUtf8("menuSimulated"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuLidar->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuFile->addAction(actionCapture);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuLidar->addAction(actionNAV200);
        menuLidar->addAction(menuSimulated->menuAction());
        menuSimulated->addAction(actionLoad_File);
        menuSimulated->addAction(actionDefault);
        menuView->addAction(actionLines);
        menuView->addAction(actionPoints);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Lidar", 0, QApplication::UnicodeUTF8));
        actionCapture->setText(QApplication::translate("MainWindow", "Capture", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionNAV200->setText(QApplication::translate("MainWindow", "NAV200", 0, QApplication::UnicodeUTF8));
        actionLoad_File->setText(QApplication::translate("MainWindow", "Load File", 0, QApplication::UnicodeUTF8));
        actionDefault->setText(QApplication::translate("MainWindow", "Default", 0, QApplication::UnicodeUTF8));
        actionLines->setText(QApplication::translate("MainWindow", "Lines", 0, QApplication::UnicodeUTF8));
        actionPoints->setText(QApplication::translate("MainWindow", "Points", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuLidar->setTitle(QApplication::translate("MainWindow", "Lidar", 0, QApplication::UnicodeUTF8));
        menuSimulated->setTitle(QApplication::translate("MainWindow", "Simulated", 0, QApplication::UnicodeUTF8));
        menuView->setTitle(QApplication::translate("MainWindow", "View", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

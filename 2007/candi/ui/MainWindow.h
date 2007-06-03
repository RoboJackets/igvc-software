#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "ui_viewer.h"

class QWidget;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	
	// Updates the contents of the video view.
	void updateVideoView();

private:
	Ui::MainWindow ui;

protected slots:
	void selectVideoView(int videoViewID);
};

#endif

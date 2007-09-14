#include "MainWindow.h"

#include "VideoView.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// Video View
	connect(ui.videoViewChooser, SIGNAL(activated(int)), SLOT(selectVideoView(int)));
	if (VideoView::getViewList()->size() >= 1) {
		selectVideoView(0);
	}
	
	const QList<VideoView *> *viewList = VideoView::getViewList();
	if (viewList) {
		for (QList<VideoView *>::const_iterator curView = viewList->begin(); curView != viewList->end(); ++curView)
		{
			ui.videoViewChooser->addItem((*curView)->getName());
		}
	}
}

void MainWindow::updateVideoView() {
	ui.videoViewWidget->update();
}

void MainWindow::selectVideoView(int videoViewID) {
	ui.videoViewWidget->view = VideoView::getViewList()->at(videoViewID);
	ui.videoViewWidget->update();
}

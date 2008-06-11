#include <QPainter>

#include "VideoWidget.h"
#include "VideoView.h"

VideoWidget::VideoWidget(QWidget *parent)
	: QWidget(parent)
{
	view = 0;
}

void VideoWidget::paintEvent(QPaintEvent *e)
{
	QPainter p(this);

	if (view) {
		view->draw(p);
	} else {
		p.fillRect(rect(), QBrush(Qt::black));
	}
}


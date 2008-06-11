#ifndef VIDEO_WIDGET_H
#define VIDEO_WIDGET_H

#include <QWidget>

class VideoView;

class VideoWidget: public QWidget
{
public:
	VideoWidget(QWidget *parent = 0);

	void paintEvent(QPaintEvent *e);

	VideoView *view;
};

#endif


#pragma once

#include <QGLWidget>

class Lidar_Thread;

class Lidar_View: public QWidget
{
	Q_OBJECT;
	
	public:
		Lidar_View(QWidget *parent = 0);
		~Lidar_View();
		
	protected:
		float _zoom;
		QPointF _center;
		QPointF _drag_start;
		Lidar_Thread *_thread;
		
		QPointF screen_to_world(QPoint pos);
		
		void paintEvent(QPaintEvent *e);
		void mousePressEvent(QMouseEvent *e);
		void mouseMoveEvent(QMouseEvent *e);
		void mouseReleaseEvent(QMouseEvent *e);
		void wheelEvent(QWheelEvent *e);
};

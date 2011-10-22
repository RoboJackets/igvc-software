#include "Lidar_View.hpp"
#include "Lidar_Thread.hpp"

#include <QPainter>
#include <QMouseEvent>

#include <math.h>
#include <stdio.h>
#include <algorithm>

using namespace std;

Lidar_View::Lidar_View(QWidget *parent):
	QWidget(parent)
{
	setAutoFillBackground(false);
	_zoom = 100;
	_thread = new Lidar_Thread(this);
	connect(_thread, SIGNAL(updated()), SLOT(update()));
	_thread->start();
}

Lidar_View::~Lidar_View()
{
	_thread->stop();
	delete _thread;
}

QPointF Lidar_View::screen_to_world(QPoint pos)
{
	return QPointF(
		(pos.x() - width() / 2) / _zoom + _center.x(),
		(height() / 2 - pos.y()) / _zoom + _center.y());
}

void Lidar_View::paintEvent(QPaintEvent *e)
{
	QPainter p(this);
	
	p.fillRect(rect(), Qt::black);
	
	p.translate(width() / 2, height() / 2);
	p.scale(_zoom, -_zoom);
	p.translate(-_center);
	
	p.setPen(Qt::red);
	p.drawLine(QPointF(0, 0), QPointF(1, 0));
	
	p.setPen(Qt::green);
	p.drawLine(QPointF(0, 0), QPointF(0, 1));

	QVector<NAV200::Point> points = _thread->points();
	QPointF pos[NAV200::Num_Points];
	for (int i = 0; i < NAV200::Num_Points; ++i)
	{
		float d = points[i].distance;
		float a = points[i].angle;
		pos[i] = QPointF(d * cos(a), d * sin(a));
	}
	
	p.setPen(Qt::white);
	for (int i = 0; i < NAV200::Num_Points; ++i)
	{
		if (points[i].valid)
		{
			int a = min(255, points[i].intensity * 255 / 50);
//			p.setPen(QColor(a, 0, 255 - a));
			
			p.drawPoint(pos[i]);
		}
	}
}

void Lidar_View::mousePressEvent(QMouseEvent *e)
{
	if (e->button() == Qt::LeftButton || e->button() == Qt::MidButton)
	{
		setCursor(Qt::ClosedHandCursor);
		_drag_start = screen_to_world(e->pos());
	}
}

void Lidar_View::mouseMoveEvent(QMouseEvent *e)
{
	if (e->buttons() & (Qt::LeftButton | Qt::MidButton))
	{
		QPointF pos = screen_to_world(e->pos());
		_center -= pos - _drag_start;
		update();
	}
}

void Lidar_View::mouseReleaseEvent(QMouseEvent *e)
{
	if (e->button() == Qt::LeftButton || e->button() == Qt::MidButton)
	{
		setCursor(Qt::ArrowCursor);
	}
}

void Lidar_View::wheelEvent(QWheelEvent *e)
{
	QPointF pos = screen_to_world(e->pos());
	float dz = pow(1.5, e->delta() / 120.0);
	_zoom *= dz;
	_center += (pos - screen_to_world(e->pos()));
	update();
}

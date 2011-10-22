#pragma once

#include <QThread>
#include <QMutex>
#include <QVector>

#include "NAV200.hpp"

class Lidar_Thread: public QThread
{
	Q_OBJECT;
	
	public:
		Lidar_Thread(QObject *parent = 0);
		
		void stop();
		
		QVector<NAV200::Point> points();
		
	signals:
		void updated();
		
	protected:
		volatile bool _run;
		QMutex _mutex;
		NAV200::Point _points[NAV200::Num_Points];
		
		virtual void run();
		virtual bool event(QEvent *e);
};

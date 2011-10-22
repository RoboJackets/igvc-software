#include "Lidar_Thread.hpp"
#include "shm_Server.hpp"

#include <QCoreApplication>

Lidar_Thread::Lidar_Thread(QObject *parent):
	QThread(parent)
{

}

void Lidar_Thread::stop()
{
	_run = false;
	wait();
}

QVector<NAV200::Point> Lidar_Thread::points()
{
	QMutexLocker lock(&_mutex);
	
	// Force a full copy
	QVector<NAV200::Point> out(NAV200::Num_Points);
	for (int i = 0; i < NAV200::Num_Points; ++i)
	{
		out[i] = _points[i];
	}

	return out;
}

void Lidar_Thread::run()
{
        NAV200 *lidar = new NAV200();
        SHM_SERVER *lidar_distance = new SHM_SERVER(LI_DISTANCE_ID);
        SHM_SERVER *lidar_angle = new SHM_SERVER(LI_ANGLE_ID);
	SHM_SERVER *lidar_intensity = new SHM_SERVER(LI_INTENSITY_ID);
       // SHM_SERVER *lidar_data = new SHM_SERVER(LI_DATA_ID);

        float _distance[NAV200::NAV200::Num_Points];
        float _angle[NAV200::Num_Points];
	float _intensity[NAV200::Num_Points];

       /* if(!lidar_data->setup(1024)){
            perror("Error setting up shared memory for LIDAR data");
            exit(0);
        }*/

        if(!lidar_distance->setup(NAV200::Num_Points)){
            perror("Error setting up shared memory for LIDAR data");
	    exit(0);

        }

        if(!lidar_angle->setup(NAV200::Num_Points)){
            perror("Error setting up shared memory for LIDAR data");
            exit(0);

        }

	if(!lidar_intensity->setup(NAV200::Num_Points)){
            perror("Error setting up shared memory for LIDAR data");
            exit(0);

        }

	_run = true;
	while (_run)
	{
		if (lidar->read())
		{
			QMutexLocker lock(&_mutex);
                        for (int i = 0; i < NAV200::Num_Points; ++i)
			{
                                _points[i] = lidar->points[i];
                                _angle[i] = lidar->points[i].angle;
                                _distance[i] = lidar->points[i].distance;
				_intensity[i] = (float)lidar->points[i].intensity;
				

			}
			
                         lidar_distance->shmWrite(_distance);
                         lidar_angle->shmWrite(_angle);
                         lidar_intensity->shmWrite(_intensity);
                    
			lock.unlock();
			
			QCoreApplication::postEvent(this, new QEvent(QEvent::User));
		} else {
			// Wait a little while so we don't spin quickly on a repeating failure
			msleep(500);
		}
	}

	delete lidar;
}

bool Lidar_Thread::event(QEvent *e)
{
	if (e->type() == QEvent::User)
	{
		updated();
	}
}

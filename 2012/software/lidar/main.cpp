#include <QtGui>

#include "Lidar_View.hpp"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	
	Lidar_View view;
	view.show();	
	
	app.exec();
	
	return 0;
}

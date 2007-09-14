#include <QApplication>
#include <libplayercore/player.h>	// for PlayerCc::PLAYER_PORTNUM

#include "main.h"

int gCameraID;
const char* gPlayerServerHostname;
int gPlayerServerPort;

int main(int argc, char *argv[])
{
	int nargs = argc-1;	
	
	/*
	 * Parse command-line arguments
	 * 
	 * syntax: cameraview [<cameraID> [<hostname> [<port>]]]
	 */
	gCameraID = (nargs >= 1) ? atoi(argv[1]) : 0;
	gPlayerServerHostname = (nargs >= 2) ? argv[2] : "localhost";
	gPlayerServerPort = (nargs >= 3) ? atoi(argv[3]) : PlayerCc::PLAYER_PORTNUM;
	
	QApplication app(/*0, NULL*/ argc, argv);
	
	MainWindow window;
	window.resize(640,480);
	window.show();
	
	return app.exec();
}

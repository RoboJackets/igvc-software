#include "main.h"
#include <QApplication>

// -----------------------------------------------------------------

#include "vision/Buffer2D.h"
#include "vision/Pixel.h"
#include "ui/VideoView.h"

// -----------------------------------------------------------------


// XXX=================
#include "CVcam.h"
// =================XXX



// -----------------------------------------------------------------

MainWindow *mainWindow;

int main(int argc, char **argv) {
	// Init the Q-App window
	QApplication app(argc, argv);
	mainWindow = new MainWindow();
		
		
	//------------------
		// Connect to robot and ALL init functions go here
		// Starting OpenGL stuff goes here in a thread
		// Starting main loop function goes here in a thread
	//------------------



	
	// XXX=================
	CVcam c;
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		c.connect( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
		c.connect(0, argv[1] );
	c.testwebcam();
	// =================XXX
	
	
	
	
	// Show the main window and run the UI thread
	mainWindow->show();
	return app.exec();
}


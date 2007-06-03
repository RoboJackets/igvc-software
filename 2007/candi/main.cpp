#include "main.h"
#include <QApplication>

// -----------------------------------------------------------------

#include "vision/Buffer2D.h"
#include "vision/Pixel.h"
#include "ui/VideoView.h"

static Buffer2D<Pixel> visRedSquareView;
static VisColorView redSquareView = VisColorView("DEBUG: Red Square View", &visRedSquareView);

// -----------------------------------------------------------------

#include "transform/transform.h"

struct TransformThreadArgs {
	int argc;
	char **argv;
};

void* transform_main_caller(void* argsOpaque) {
	TransformThreadArgs* args = (TransformThreadArgs*) argsOpaque;
	
	transform_main(args->argc, args->argv);
	
	delete args;
	return NULL;
}

// -----------------------------------------------------------------

MainWindow *mainWindow;

int main(int argc, char **argv) {
	QApplication app(argc, argv);
	mainWindow = new MainWindow();
	
	// Create red square view
	visRedSquareView.resize(50,50);
	for (int i=0, n=visRedSquareView.numElements(); i<n; i++) {
		visRedSquareView[i] = Pixel(255,0,0);
	}
	
	// Start the thread that takes in data from the camera,
	// transforms it, and hands off the data for vision processing.
	pthread_t transformThread;
	TransformThreadArgs* transformThreadArgs = new TransformThreadArgs();
	transformThreadArgs->argc = argc;
	transformThreadArgs->argv = argv;
	pthread_create(&transformThread, NULL, transform_main_caller, transformThreadArgs);
	
	// XXX: Wait for the "transform" window to open
	usleep(5000 * 100);		// 5000 ms
	
	// Show the main window and run the UI thread
	mainWindow->show();
	return app.exec();
}


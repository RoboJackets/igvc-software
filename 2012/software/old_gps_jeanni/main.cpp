#include "main.h"
#include "Robot.h"
#include <QApplication>

int main(int argc, char **argv)
{

	(void)argc; // avoid unused variable warnings

	/*
	 * Our display window
	 * This has a trackbar at the top to select views
	 */
	cvNamedWindow("display", 0);

	/*
	 * Start robot main loop in a thread
	 * (do vision processing, sensors, and drive control)
	 */
	//char* input = (argc > 1) ? argv[1] : NULL

	Robot r(argv[1]);
	r.startRobotThread(&r); // this doesn't return

	return 0;
}


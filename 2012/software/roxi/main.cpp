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
	Robot r(argv[1]);
	r.Go(); // start the robot
	r.destroy(); // kill the robot;

	return 0;
}


#include "main.h"
#include "Robot.h"
#include <QApplication>

int main(int argc, char **argv) {

    /*
     * Our display window
     * This has a trackbar at the top to select views
     */
    cvNamedWindow("display", NULL);

    /*
     * Start robot main loop in a thread
     * (do vision processing, sensors, and drive control)
     */
    Robot r(argv[1]);
    r.startRobotThread(&r); // this doesn't return

    return 0;
}


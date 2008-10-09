#include "main.h"
#include <QApplication>
#include "Robot.h"

///////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////



int main(int argc, char **argv) {


    //	/////////////////////////////////////////////////////////////////////


    //	////////////////////////////////////////////////////////////////////

    /*
     * ALL other init functions go here
     */

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


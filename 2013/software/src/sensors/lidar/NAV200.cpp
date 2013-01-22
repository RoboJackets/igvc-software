/*
 * NAV200.cpp
 *
 *  Created on: Jan 22, 2012
 *      Author: Alexander Huynh
Copypasta from: Matthew Barulic
 */

#include "NAV200.h"
#include <iostream>
#include <string>

namespace IGVC {
namespace Sensors {

/*NAV200::NAV200():
    serialPort("/dev/NAV200", 9600),
	iothread(boost::bind( &NAV200::threadRun, this)),*/

NAV200::NAV200()                                                //ERR: no matching function for call to 'ASIOSerialPort::ASIOSerialPort()'
{
    //serialPort("/dev/NAV200", 9600);
	//iothread(boost::bind( &NAV200::threadRun, this));
}


void NAV200::threadRun() {
	while(serialPort.isConnected()) {
		/*

		//input appears to be in some form containing 1024 pairings of this data:
            x from -6.27705 to 0                angle, radians starting toward the right and going CCW
            y from 0 to Lidar's max range       distance, unknown units

            input is stored in an array of Points (defined in Lidar.h)


        //old years' code read:
            uint8_t data[3083];
            memset(data, 0, sizeof(data));
            int ret = usb_control_msg(_handle, 0xc0, 4, 0, 0, (char *)data, sizeof(data), 500);

        //new code ?
            data = serialPort.readln();     //get input, supposedly in the form of an array

        //old years' code read:
            int start = data[5] + data[6] * 256;
            int offset = 7;
            for (int i = 0; i < Num_Points; ++i)
            {
                int ia = (i + start) % Num_Points;
                Point &pt = points[ia];

                pt.raw = data[offset] + data[offset + 1] * 256;
                pt.distance = pt.raw * distance_scale;
                pt.intensity = data[offset + 2];
                pt.valid = !(pt.raw & 0x4000);
                pt.angle = -ia * 2 * M_PI / NAV200::Num_Points;

                offset += 3;

                valid[i] = pt.valid;
                if(pt.valid)
                {
                    theta[i] = pt.angle;
                    radius[i] = pt.distance;
                    //polar2cart(pt.angle, pt.distance, x[i], y[i]);
                }
                else
                {
                    radius[i] = theta[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
		*/
	}
}

NAV200::~NAV200() {
    serialPort.close();
}

} /* namespace Sensors */
} /* namespace IGVC */

/*
 * NAV200.cpp
 *
 *  Created on: Jan 22, 2012
 *      Author: Alexander Huynh
 *    Coauthor: Matthew Barulic
 */
#include "NAV200.h"
#include <iostream>
#include <string>
#include <libusb-1.0/libusb.h>
#include <cmath>

using namespace std;

namespace IGVC {
namespace Sensors {

NAV200::NAV200()
{
    _numPoints = 1024;

    _ctx = NULL;

    int err = libusb_init(&_ctx);
    if( err < 0 ) {
        cout << "Init Error " << err << endl;
        return;
    }

    libusb_set_debug(_ctx, 3);
    _handle = libusb_open_device_with_vid_pid(_ctx, 12609, 8);

    _running = true;
    _iothread = boost::thread(boost::bind( &NAV200::threadRun, this));
}

LidarState NAV200::GetState()
{
    return LidarState();
}

LidarState NAV200::GetStateAtTime(timeval time)
{
    return LidarState();
}

bool NAV200::StateIsAvailable()
{
    return true;
}


void NAV200::threadRun() {
	while (_running) {
        uint8_t data[3083];
        memset(data, 0, sizeof(data));

        int ret = libusb_control_transfer(_handle, 0xc0, 4, 0, 0, data, sizeof(data), 500);
        if(ret != sizeof(data))
        {
            cerr << "Error in read, returned : " << ret << endl;
            switch(ret)
            {
                case LIBUSB_ERROR_TIMEOUT:
                    cout << "Timeout" << endl;
                    break;
                case LIBUSB_ERROR_PIPE:
                    cout << "Pipe" << endl;
                    break;
                case LIBUSB_ERROR_NO_DEVICE:
                    cout << "no device"<<endl;
                    break;
            }
        } else {
            uint8_t checksum = 0;
            for (int i = 0; i < (ret - 1); ++i)
            {
                checksum += data[i];
            }
            if (checksum != data[ret - 1])
            {
                cout << "Bad checksum!" << endl;
            } else {
                LidarState state;
                float distance_scale =  0.8255 / 300.0;
                int start = data[5] + data[6] * 256;
                int offset = 7;
                for(int i=0; i < _numPoints; i++)
                {
                    int ia = (i+start) % _numPoints;
                    LidarPoint &pt = state.points[ia];

                    // Converting from little endian data
                    pt.raw = data[offset] + ( data[offset + 1] << 8 );
                    // Derived this distance formula from page 14 of
                    // "Telegrams for Configuring and Operating the
                    // NAV200 Laser Positioning System"
                    pt.distance = ( ( pt.raw / 65536.0 ) * 29900.0 ) + 100.0;
                    pt.distance /= 1000.0; // convert mm to m
                    pt.intensity = data[offset + 2];
                    pt.valid = !(pt.raw & 0x4000);
                    pt.angle = -ia * ( 2 * M_PI / _numPoints );

                    offset += 3;
                }
                onNewData(state);
            }
        }

	}

    libusb_close(_handle);
    libusb_exit(_ctx);
    cout << "closed!" << endl;
	return;
}

void NAV200::stop()
{
    _running = false;
    _iothread.join();
}

NAV200::~NAV200() {
}

} /* namespace Sensors */
} /* namespace IGVC */

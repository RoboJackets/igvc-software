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

bool NAV200::IsWorking()
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
                int start = data[5] + data[6] * 256;
                int offset = 7;
                for(int i=0; i < _numPoints; i++)
                {
                    int ia = (i+start) % _numPoints;
                    LidarPoint &pt = state.points[ia];

                    // Converting from little endian data
                    pt.raw = data[offset] + ( data[offset + 1] << 8 );

                    pt.distance = pt.raw / 391.8181818;
                    pt.intensity = data[offset + 2];
                    pt.valid = !(pt.raw & 0x4000);
                    pt.angle = -ia * ( 2 * M_PI / _numPoints ) + 1.4 - M_PI/2.0;

                    offset += 3;
                }

                try {
                    boost::this_thread::interruption_point();
                } catch (boost::thread_interrupted&)
                {
                    break;
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

NAV200::~NAV200() {
    _running = false;
    _iothread.interrupt();
}


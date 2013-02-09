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

namespace IGVC {
namespace Sensors {

NAV200::NAV200()
{
    ctx = NULL;

    int r;
    r = libusb_init(&ctx);

    if(r < 0) {
        cout << "Init Error " << r << endl;
        return;
    }

    libusb_set_debug(ctx, 3);
    _handle = libusb_open_device_with_vid_pid(ctx, 12609, 8);

    _running = true;
    iothread = boost::thread(boost::bind( &NAV200::threadRun, this));
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
                float distance_scale =  0.8255 / 300.0;
                int start = data[5] + data[6] * 256;
                int offset = 7;
                for(int i=0; i < 1024; i++)
                {
                    int ia = (i+start) % 1024;
                    uint16_t raw = data[offset] + data[offset + 1] * 256;
                    bool valid = !(raw & 0x4000);
                    if(valid)
                    {
                        cout << raw * distance_scale << "m @";
                        cout << -ia * 2 * M_PI / 1024 << "deg" << endl;
                    }
                    offset += 3;
                }
            }
        }

	}

    libusb_close(_handle);
    libusb_exit(ctx);
    cout << "closed!" << endl;
	return;
}

void NAV200::stop()
{
    _running = false;
    iothread.join();
}

NAV200::~NAV200() {
}

} /* namespace Sensors */
} /* namespace IGVC */

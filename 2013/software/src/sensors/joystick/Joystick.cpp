#include "Joystick.h"

bool Joystick::initJoystick()
{
    //The Logitech
    joy_fd = open("/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-joystick", O_RDONLY); //Open Joystick file for read-only

    //The XBox
    //joy_fd = open("/dev/input/js0", O_RDONLY); //Open Joystick file for read-only
    if(joy_fd < 0)
    {
        cout << "Could not open joystick" << endl;
        return false;
    }

    else
    {
        cout << "Joystick opened." << endl;
        cout << "Press ctrl+c to quit." << endl;
        numAxes = 0, numButtons = 0;
        ioctl(joy_fd, JSIOCGAXES, &numAxes );
        ioctl( joy_fd, JSIOCGBUTTONS, &numButtons );

        axis.resize(numAxes);
        B.resize(numButtons);
        leftYAxis=0; leftXAxis=0; rightYAxis=0; rightXAxis=0;
        padYAxis=0; padXAxis=0;
        return true;
    }
}

Joystick::~Joystick()
{
    close(joy_fd);
    joy_fd = -1;
    axis.clear();
    B.clear();
}

bool Joystick::readJoystick()
{
        struct pollfd pfd;

        pfd.fd = joy_fd;

        pfd.events = POLLIN | POLLHUP;

        int ret = poll(&pfd, 1, 0);

        if (ret == 0)
        {
            //no data
            return false;
        }
        else if(ret < 0)
        {
            // Poll error (not fd error)
            if(errno != EINTR)
            {
                cout << "Joystick pull error!" << endl;
                return false;
            }
        }

        struct js_event event; //The Joystick Event Structure

        //Read Joystick Event
        if(read(joy_fd, &event, sizeof(event)) != sizeof(event))
        {
            return false;
        }

        //Assign Button Values
        if (event.type == JS_EVENT_BUTTON)
        {
            B[event.number] = event.value;
        }

        //Assign Analog Axes values and Pad values
        else if (event.type == JS_EVENT_AXIS)
        {
            axis[event.number] = event.value;
        }
        leftXAxis=axis[0]; leftYAxis=-axis[1]; rightXAxis=axis[2]; rightYAxis=-axis[3];
        padXAxis=axis[4]; padYAxis=-axis[5];

        return true;


}

void Joystick::displayJoystick()
{

    cout<<" leftXAxis : "   <<leftXAxis<<"    ";
    cout<<" leftYAxis : "   <<leftYAxis<<"    ";
    cout<<" rightXAxis : "  <<rightXAxis<<"    ";
    cout<<" rightYAxis : "  <<rightYAxis<<"    ";
    cout<<" padXAxis : "    <<padXAxis<<"    ";
    cout<<" padYAxis : "    <<padYAxis<<"    ";

    for(unsigned int i = 0; i < B.size(); i++)
        {
            cout << "Button" << i+1 << ": " << B[i] << "    ";
        }

    cout<<endl;

}

#include "Joystick.h"

bool Joystick::initJoystick() {
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
        LYAxis=0; LXAxis=0; RYAxis=0; RXAxis=0;
        padYAxis=0; padXAxis=0;
        jLPWM=255; sLPWM=255; LPWM=0;
        jRPWM=255; sRPWM=255; RPWM=0;
        LDirection=0; RDirection=0;
        return true;
    }
}

Joystick::Joystick() {
   // initJoystick();
}

Joystick::~Joystick() {
    close(joy_fd);
    joy_fd = -1;
    axis.clear();
    B.clear();
}

void Joystick::startEvents() {
    _eventsEnabled = true;
	eventThread = boost::thread(boost::bind(&Joystick::eventThreadRun, this));
}

void Joystick::stopEvents() {
    _eventsEnabled = false;
}

void Joystick::eventThreadRun() {
    while(_eventsEnabled)
    {
            //Put Main Loop from test file here after complete debugging
            //Code will not have main test file.
    }
}

bool Joystick::readJoystick() {
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
        LXAxis=axis[0]; LYAxis=-axis[1]; RXAxis=axis[2]; RYAxis=-axis[3];
        padXAxis=axis[4]; padYAxis=-axis[5];

        return true;


}

void Joystick::getPWM() {

            jLPWM=(LYAxis/32768.0*255)+255;
            jRPWM=(RYAxis/32768.0*255)+255;

}

void Joystick::smoothPWM() {


//    if(LDirection==sLDIRECTION)
//    {
//        if(LPWM>sLPWM)
//        {
//            sLPWM+=1;
//        }
//
//        else if(LPWM<sLPWM)
//        {
//            sLPWM-=1;
//        }
//    }
//
//    else if(LDirection>sLDIRECTION)
//    {
//        if(LPWM>sLPWM)
//        {
//            sLPWM+=1;
//        }
//
//        else if(LPWM<sLPWM)
//        {
//            sLPWM-=1;
//        }
//    }
//
//    else if(LDirection<sLDIRECTION)
//    {
//        if(LPWM>sLPWM)
//        {
//            sLPWM+=1;
//        }
//
//        else if(LPWM<sLPWM)
//        {
//            sLPWM-=1;
//        }
//    }

    //Smooth Left Motor PWM - de-offset and set direction
    if(jLPWM-5>sLPWM)
    {
        sLPWM+=5;
        if(sLPWM>510)
        {
            sLPWM=510;
        }
        LPWM=sLPWM-255;
        LDirection=signbit(LPWM);
        if(LDirection==1)
        {
            LPWM=255-LPWM;
        }
    }
    else if(jLPWM+5<sLPWM)
    {
        sLPWM-=15;
        if(abs(sLPWM)-255<=15)
        {
            sLPWM=255;
        }

        LPWM=sLPWM-255;
        LDirection=signbit(LPWM);
        if(LDirection==1)
        {
            LPWM=255-LPWM;
        }
    }

    //Smooth Right Motor PWM - de-offset and set direction
    if(jRPWM-5>sRPWM)
    {
        sRPWM+=5;
        if(sRPWM>510)
        {
            sRPWM=510;
        }
        RPWM=sRPWM-255;
        RDirection=signbit(RPWM);
        if(RDirection==1)
        {
            RPWM=255-RPWM;
        }
    }
    else if(jRPWM+5<sRPWM)
    {
        sRPWM-=15;
        if(abs(sRPWM)-255<=15)
        {
            sRPWM=255;
        }
        RPWM=sRPWM-255;
        RDirection=signbit(RPWM);
        if(RDirection==1)
        {
            RPWM=255-RPWM;
        }
    }


}

bool Joystick::CycleCheck(){

    gettimeofday(&cycleTime, NULL);

    currentMicro = cycleTime.tv_usec;
    //cout<<"currentMicro"<<currentMicro<<"\n";
    //cout<<"diffMicro"<<currentMicro-lastMicro<<"\n";
    if(currentMicro - lastMicro >= setCYCLE)
    {
        lastMicro = currentMicro;
        cout<<"Cyclesuccess"<<"\n";
        return true;
    }
    else
    {
        return false;
    }
}

void Joystick::displayJoystick() {

    cout<<" leftXAxis : "   <<LXAxis<<"    ";
    cout<<" leftYAxis : "   <<LYAxis<<"    ";
    cout<<" rightXAxis : "  <<RXAxis<<"    ";
    cout<<" rightYAxis : "  <<RYAxis<<"    ";
    cout<<" padXAxis : "    <<padXAxis<<"    ";
    cout<<" padYAxis : "    <<padYAxis<<"    ";

    for(unsigned int i = 0; i < B.size(); i++)
        {
            cout << "Button" << i+1 << ": " << B[i] << "    ";
        }

    cout<<endl;

}

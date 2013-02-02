#include "Joystick.h"
#include <string>

//Initialize Global Motion Control Variables
    //Used by Single Stick Drive Mode
    float Speed=0;
    float Heading=0.0;
    //Used by Dual Stick Drive Mode
    int leftSpeed=0, rightSpeed=0;
    int leftDirection=0, rightDirection=0;

/////////////////////////////

bool Joystick::initJoystick()
{
    joystick=NULL;

    std::cout<<"Initializing Joystick Subsystem"<<"\n";

    //Initialize SDL joystick subsystem
    if( SDL_Init( SDL_INIT_JOYSTICK ) == -1 )
    {
        std::cout<<"Error Initializing Joystick Subsystem"<<"\n";
        return false;
    }
    std::cout<<"Joystick Subsystem Initialized"<<"\n";
    // make sure SDL cleans up before exit
    atexit(SDL_Quit);
    //Check # of joysticks connected
    if( SDL_NumJoysticks() < 1 )
    {
        std::cout<<"No Joystick Found"<<"\n";
        return false;
    }
    std::cout<<SDL_NumJoysticks()<<"Joystick/s found"<<"\n";

    //Open joystick/s
    joystick = SDL_JoystickOpen( 0 );

    //If there's a problem opening the joystick
    if( joystick == NULL )
    {
    	std::cout<<"Error Opening Joystick"<<"\n";
        return false;
    }

    //Initialize joystick Variables
    leftAnalogX=0; leftAnalogY=0;
    rightAnalogX=0; rightAnalogY=0;
    dPadX=0; dPadY=0; dPadA=0; dPadB=0;
    DigitalUp=0; DigitalLeft=0; DigitalRight=0; DigitalDown=0;
    SENSITIVITY=4000;

    //If everything is initialized
    return true;
}void Joystick::readJoystick()
{
// TODO (ali#1#): Where does this "SDL_event" go? In function or in Main?    SDL_Event joyevent;
    int axisValue;

    while ( SDL_PollEvent(&joyevent) )
    {

    switch(joyevent.type) {
        case SDL_JOYAXISMOTION:
                //Zero axis value if below sensitivity threshold.
                if (abs(joyevent.jaxis.value)<SENSITIVITY)
                axisValue=0;
                else
                axisValue=joyevent.jaxis.value;
            switch (joyevent.jaxis.axis)   {
                case 0:
                    leftAnalogX = axisValue;
                    break;
                case 1:
                    leftAnalogY = axisInvert*axisValue;
                    break;
                case 3:
                    rightAnalogX = axisValue;
                    break;
                case 4:
                    rightAnalogY = axisInvert*axisValue;
                    break;
// TODO (ali#1#): Add other Joystick axis/bottons
                default:
                    break;
                                        }
// TODO (ali#1#): Add Button Actions
//        case SDL_JOYBUTTONDOWN:
//            //joystickButtons |= (1 << joyevent.jbutton.button);
//            break;
//        case SDL_JOYBUTTONUP:
//            //joystickButtons &= ~(1 << joyevent.jbutton.button);
//            break;
        case SDL_QUIT:
        Quit();

                        }
    }


}


void Joystick::setMotion()
{
    double axisMax=327.68;

    //Mode 1: Single Stick Control
    Speed=sqrt(pow(leftAnalogX,2)+pow(leftAnalogY,2))/axisMax;
    if (leftAnalogX == 0.0 && leftAnalogY==0.0)
    Heading=0.0;
    else if (leftAnalogX == 0.0 && leftAnalogY!=0.0)
    Heading=PI/2*(2-leftAnalogY/abs(leftAnalogY));
    else
    Heading=atan(-leftAnalogY/leftAnalogX);
    //Speed=1;
    //Heading=10;
    //std::cout<<"Speed   = "<<Speed<<"% \n";
    //std::cout<<"Heading = "<<Heading<<" rad\n";

    //Mode 2: Dual Stick Control

    leftSpeed=abs(leftAnalogY)/axisMax;
    rightSpeed=abs(rightAnalogY)/axisMax;

    if(leftAnalogY<0)
    leftDirection=-1;
    else if(leftAnalogY>0)
    leftDirection=1;
    else
    leftDirection=0;

    if(rightAnalogY<0)
    rightDirection=-1;
    else if(rightAnalogY>0)
    rightDirection=1;
    else
    rightDirection=0;
}

void Joystick::cleanJoystick()
{

    //Close the joystick
    SDL_JoystickClose( joystick );

    //Quit SDL
    SDL_Quit();
}

bool Joystick::Quit()
{
    return true;
}
/////////////////////////////

bool Graph::initGraph()
{
    screen=NULL;
    compass=NULL;
    leftspeedlabel=NULL;
    rightspeedlabel=NULL;
    gaugeframe=NULL;
    gaugebar=NULL;
    arrow=NULL;

    std::cout<<"Initializing Video Subsystem"<<"\n";
    //Initialize SDL video subsystems
    if( SDL_Init( SDL_INIT_VIDEO ) == -1 )
        {
            std::cout<<"Error Initializing Video Subsystem"<<"\n";
            return false;
        }

    std::cout<<"Video Subsystem Initialized"<<"\n";


    //Set up the screen
    screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

    //If there was an error in setting up the screen
    if( screen == NULL )
    {
        std::cout<<"Error Setting Up Screen"<<"\n";
        return false;
    }

    //Set the window caption
    SDL_WM_SetCaption( "ODOMETER", NULL );

    //Fill the screen white
    SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0xFF, 0xFF, 0xFF ) );

    // Initialize SDL font subsystem
    if (TTF_Init() != 0)
    {
      std::cout << "Error Initializing Font Library " << TTF_GetError() <<"\n";
      return false;
    }

    font = TTF_OpenFont("src/sensors/joystick/FreeSans.ttf", 24);
    if (font == NULL)
    {
      std::cout<< "Error Opening Font " << TTF_GetError() <<"\n";
      return false;
    }

    if(loadGraph()== NULL)
    {
        std::cout << "Error Loading Graph" << TTF_GetError() <<"\n";
        return false;
    }


    //The offsets of the Compass
    xCompass=160; yCompass=240;
    //The offsets of the Left Gauge frame
    xleftGaugeframe=330; yleftGaugeframe=20;
    //The offsets of the Right Gauge frame
    xrightGaugeframe=500; yrightGaugeframe=20;
    //The offsets of the left Gauge bar
    xleftGaugebar=315; yleftGaugebar=150;
    //The offsets of the Right Gauge bar
    xrightGaugebar=485; yrightGaugebar=150;
    //The offsets of the Left Speed Label
    xleftLabel=300; yleftLabel=320;
    //The offsets of the Right Speed Label
    xrightLabel=460; yrightLabel=320;
    //The offsets of the left Speed#
    xleftSpeed=350; yleftSpeed=360;
    //The offsets of the right Speed#
    xrightSpeed=510; yrightSpeed=360;
    //The offsets of the Arrow Center
    xArrow=160; yArrow=240;
    thArrow=PI/2;
    //If everything initialized fine
    return true;
}

SDL_Surface *OptimizeImage( std::string filename )
{
    //The image that's loaded
    SDL_Surface* loadedImage = NULL;

    //The optimized surface that will be used
    SDL_Surface* optimizedImage = NULL;

    //Load the image
    loadedImage = IMG_Load( filename.c_str() );

    //If the image loaded
    if( loadedImage != NULL )
    {
        //Create an optimized surface
        optimizedImage = SDL_DisplayFormat( loadedImage );

        //Free the old surface
        SDL_FreeSurface( loadedImage );

        //If the surface was optimized
        if( optimizedImage != NULL )
        {
            //Color key surface
            SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, SDL_MapRGB( optimizedImage->format, 0, 0xFF, 0xFF ) );
        }
    }

    //Return the optimized surface
    return optimizedImage;
}

bool Graph::loadGraph()
{
    //Load the images
    arrow = OptimizeImage( "src/sensors/joystick/arrow.bmp" );
    compass = OptimizeImage( "src/sensors/joystick/compass.bmp" );
    leftspeedlabel = OptimizeImage( "src/sensors/joystick/leftspeedlabel.bmp" );
    rightspeedlabel = OptimizeImage( "src/sensors/joystick/rightspeedlabel.bmp" );
    gaugeframe = OptimizeImage( "src/sensors/joystick/gaugeframe.bmp" );
    gaugebar = OptimizeImage( "src/sensors/joystick/gaugebar.bmp" );
    //If there was a problem in loading the dot
    if( arrow == NULL && compass == NULL && leftspeedlabel == NULL && rightspeedlabel == NULL)
    {
        std::cout<<"Error Loading Images"<<"\n";
        return false;
    }

    return true;
}


void Graph::displayGraph()
{
    // message processing loop
        SDL_Event screenevent;
        while (SDL_PollEvent(&screenevent))
        {
            // check for messages
            switch (screenevent.type)
            {
                // exit if the window is closed
            case SDL_QUIT:
            // done = true;
                    break;
                break;

                // check for keypresses
            case SDL_KEYDOWN:
                {
                    // exit if ESCAPE is pressed
                    if (screenevent.key.keysym.sym == SDLK_ESCAPE)
                       // done = true;
                    break;
                }
            } // end switch
        } // end of message processing

    int flipstatus;
    //Fill the screen white
    SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0xFF, 0xFF, 0xFF ) );

    //Apply images to screen
    //applyImage(xArrow,yArrow, arrow, screen, NULL);
    //applyImage(xCompass,yCompass, compass, screen, NULL);
    applyImage(xleftLabel,yleftLabel, leftspeedlabel, screen, NULL);
    applyImage(xrightLabel,yrightLabel, rightspeedlabel, screen, NULL);
    applyImage(xleftGaugeframe,yleftGaugeframe, gaugeframe, screen, NULL);
    applyImage(xrightGaugeframe,yrightGaugeframe, gaugeframe, screen, NULL);
    yleftGaugebar=(150-leftDirection*leftSpeed);
    applyImage(xleftGaugebar,yleftGaugebar, gaugebar, screen, NULL);
    yrightGaugebar=(150-rightDirection*rightSpeed);
    applyImage(xrightGaugebar,yrightGaugebar, gaugebar, screen, NULL);

    //Apply changing text to screen
    applyText(xleftSpeed,yleftSpeed,leftDirection*leftSpeed,screen, NULL);
    applyText(xrightSpeed,yrightSpeed,rightDirection*rightSpeed,screen, NULL);
    //Update the screen
    flipstatus=SDL_Flip(screen);
    //std::cout << "SDL Flip Status= "<<flipstatus<<"\n";
}

void Graph::applyImage(int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip)
{
    int blitstatus;
    //Holds offsets
    SDL_Rect offset;
    clip = NULL;
    //Get offsets
    offset.x = x;
    offset.y = y;
    //Blit
    blitstatus=SDL_BlitSurface(source, clip, destination, &offset );

}

void Graph::applyText(int x, int y, double s, SDL_Surface* destination, SDL_Rect* clip)
{
    //Holds offsets
    SDL_Rect offset;
    clip = NULL;
    SDL_Color text_color = {0, 191, 255};
    std::stringstream speedString;
    speedString<<s;
    //source = TTF_RenderText_Solid(font,"Hello", text_color);
    source = TTF_RenderText_Solid(font,speedString.str().c_str(), text_color);
    //Get offsets
    offset.x = x;
    offset.y = y;
    //Blit
    SDL_BlitSurface(source, clip, destination, &offset );

}


void Graph::cleanGraph()
{
    //Free the surface
    SDL_FreeSurface( screen );
    SDL_FreeSurface( compass );
    SDL_FreeSurface( leftspeedlabel );
    SDL_FreeSurface( rightspeedlabel );
    SDL_FreeSurface( arrow );
    SDL_FreeSurface( leftspeedtext );

    //Quit SDL
    SDL_Quit();
}

bool Graph::Quit()
{
    return true;
}
////////////////////////////


bool Timer::initTimer()
{
    if( SDL_Init( SDL_INIT_TIMER ) == -1 )
        {
    	std::cout<<"SDL_INIT TIMER Fail "<<SDL_GetError()<<"\n";
    	return false;
        }

    std::cout<<"SDL Timer Initialized"<<"\n";

    //Initialize the variables
    startTicks = 0;
    pausedTicks = 0;
    paused = false;
    started = false;
}

void Timer::start()
{
    //Start the timer
    started = true;

    //Unpause the timer
    paused = false;

    //Get the current clock time
    startTicks = SDL_GetTicks();
}

void Timer::stop()
{
    //Stop the timer
    started = false;

    //Unpause the timer
    paused = false;
}

void Timer::pause()
{
    //If the timer is running and isn't already paused
    if( ( started == true ) && ( paused == false ) )
    {
        //Pause the timer
        paused = true;

        //Calculate the paused ticks
        pausedTicks = SDL_GetTicks() - startTicks;
    }
}

void Timer::unpause()
{
    //If the timer is paused
    if( paused == true )
    {
        //Unpause the timer
        paused = false;

        //Reset the starting ticks
        startTicks = SDL_GetTicks() - pausedTicks;

        //Reset the paused ticks
        pausedTicks = 0;
    }
}

int Timer::get_ticks()
{
    //If the timer is running
    if( started == true )
    {
        //If the timer is paused
        if( paused == true )
        {
            //Return the number of ticks when the timer was paused
            return pausedTicks;
        }
        else
        {
            //Return the current time minus the start time
            return SDL_GetTicks() - startTicks;
        }
    }

    //If the timer isn't running
    return 0;
}

bool Timer::is_started()
{
    return started;
}

bool Timer::is_paused()
{
    return paused;
}





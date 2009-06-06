
#include "joystickDrive.hpp"

//static SDL_Joystick *joystick;

int main()
{
	joystickDrive jD;

	for(;;)
	{
		jD.setMotor();
		usleep(1000);
		
	}
}

joystickDrive::joystickDrive()
{
	joystick_open();
	m_motorCtr.SetupSerial();
}

joystickDrive::~joystickDrive()
{
	joystick_close();
}


//return [0,2 PI], normal math def
inline double joystickDrive::getHeading()
{
	//double theta = atan2(-1*rightAnalogY, rightAnalogX);
	double theta = atan2(-1*leftAnalogY, leftAnalogX);
 
	return theta;
}

inline double scaleThrottle(double t)
{
	const double slope = (double)100/(double)255;
	std::cout << "t: " << t << "\tf(t):" << (t-50)*slope + 50 << std::endl;
	return( (t-50)*slope + 50 );
}

void joystickDrive::setMotorTank()
{
	readJoystick();	
	int lvel = (-1*leftAnalogY+32768) / (double)65536 * (double)255-128;
	int rvel = (-1*rightAnalogY+32768) / (double)65536 * (double)255-128;
	std::cout << "init scale lvel: "<< lvel << std::endl;
	if( (abs(lvel) - 50 < 0) || isnan(lvel) || isinf(lvel))//dead zone
	{
		lvel = 0;
		std::cout << "first" << std::endl;
	}
	else if(lvel > 127 || lvel < -128)//stupid input check ("range check")
	{
		lvel = 0;
		std::cout << "second" << std::endl;
	}
	else if( (abs(scaleThrottle(lvel)) > 127) )//scale check
	{
		lvel = 127;
	}
	else
	{
		lvel = scaleThrottle(lvel);
	}

	if( (abs(rvel) - 50 < 0) || isnan(rvel) || isinf(rvel))//dead zone
	{
		rvel = 0;
	}
	else if(rvel > 127 || rvel < -128)//stupid input check ("range check")
	{
		rvel = 0;
	}
	else if( (abs(scaleThrottle(rvel)) > 127) )//scale check
	{
		rvel = 127;
	}
	else
	{
		rvel = scaleThrottle(rvel);
	}
	std::cout << "right at ("<< rightAnalogX << "," << rightAnalogY << ")\trvel: " << rvel << std::endl;
	std::cout << "left at ("<< leftAnalogX << "," << leftAnalogY << ")\tlvel: " << lvel << std::endl;
	m_motorCtr.set_motors(lvel, rvel);
}

void joystickDrive::setMotor()
{
	readJoystick();
	char theta = -1*(getHeading() * 128/(M_PI) + 64 + 128);

	//double mag = sqrt(rightAnalogX*rightAnalogX + rightAnalogY*rightAnalogY);
	double mag = sqrt(leftAnalogX*leftAnalogX + leftAnalogY*leftAnalogY);
	double norm_mag = mag / (32768*sqrt(2));
	double vel = norm_mag * 255;

	if(vel < 50 || isnan(vel) || isinf(vel) || vel > 255)//dead zone
	{
		vel = 0;
		theta = 0;
	}
	else if((vel > 128) && (scaleThrottle(vel) > 128))//max
	{
		vel = 128;
	}
	else if( theta > 120 || theta < -120 )
	{
		vel = scaleThrottle(vel);
		m_motorCtr.set_motors(-1*vel, -1*vel);
	}
	else
	{
		vel = scaleThrottle(vel);
	}

	//std::cout << "right at ("<< rightAnalogX << "," << rightAnalogY << ")\twould have set vel: " << vel << " head: " << (int)theta << std::endl;
	std::cout << "right at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set vel: " << vel << " head: " << (int)theta << std::endl;

	m_motorCtr.set_heading(vel,theta);
}

void joystickDrive::joystick_close(void) {
	// Send an SDL_QUIT event to the joystick event loop
	SDL_QuitEvent quitEvent = { SDL_QUIT };
	SDL_PushEvent((SDL_Event*) &quitEvent);
}

/**
 * Opens the joystick.
 * 
 * @return		NULL if success;
 * 				or a message if error.
 */
const char* joystickDrive::joystick_open(void) {
	int err;
	
	// Initialize SDL
	if ( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0 ) {
		return SDL_GetError();
	}
	
	joystick = SDL_JoystickOpen(0);
	if (!SDL_JoystickOpened(0)) {
		// TODO: would it work to return SDL_GetError() here?
		return "SDL_JoystickOpen failed";
	}
	
	// Call joystick_run() in new thread
	//err = pthread_create(&joystick_thread, NULL, &joystick_run, NULL);
	switch (err) {
		case EAGAIN:
			return "insufficient thread resources";
		case EINVAL:
			return "invalid thread attributes";
		default:
			break;
	}
	
	return NULL;	// no error
}

void joystickDrive::readJoystick() {
	int i;
	int done;
	SDL_Event event;
	
	/*
	// Ignore initial events during startup
	for (i=0;i<20;i++) {
		SDL_PollEvent(&event);
	}
	*/
	// Poll the joystick for events, and update globals
	done = 0;
		while ( SDL_PollEvent(&event) ) {
			switch (event.type) {
				case SDL_JOYAXISMOTION:
					switch (event.jaxis.axis) {
						case 0:
							leftAnalogX = event.jaxis.value;
							break;
						case 1:
							leftAnalogY = event.jaxis.value;
							break;
						case 2:
							rightAnalogX = event.jaxis.value;
							break;
						case 3:
							rightAnalogY = event.jaxis.value;
							break;
						case 4:
							dPadX = event.jaxis.value;
							break;
						case 5:
							dPadY = event.jaxis.value;
							break;
						default:
							break;
					}
					break;
				case SDL_JOYBUTTONDOWN:
					joystickButtons |= (1 << event.jbutton.button);
					break;
				case SDL_JOYBUTTONUP:
					joystickButtons &= ~(1 << event.jbutton.button);
					break;
				/*
				case SDL_KEYDOWN:
					if ( event.key.keysym.sym != SDLK_ESCAPE ) {//this line doesnt work
						break;
					}
					// Fall through to signal quit
				*/
				case SDL_QUIT:
					done = 1;
					break;
				default:
					break;
			}
		}
}

#include "joystickDrive.hpp"

//static SDL_Joystick *joystick;
/*
//Main now in joystickmain
int main()
{
	joystickDrive jD;

	for(;;)
	{
		//jD.printLoop();		
		jD.setMotor();
		jD.readJoystick();
		if(jD.shouldQuit())
		{
			return(0);
		}
		usleep(4e4);
		
	}
}
*/
joystickDrive::joystickDrive()
{
	//m_motorCtr = new OSMC_driver();	
	m_motorCtr = new OSMC_4wd_driver();

	quit = false;
	leftAnalogX = 0;
	leftAnalogY = 0;
	rightAnalogX = 0;
	rightAnalogY = 0;
	dPadX = 0;
	dPadY = 0;
	joystickButtons = 0;

	joystick_open();
	
}

joystickDrive::joystickDrive(OSMC_4wd_driver* osmc)
{		
	m_motorCtr = osmc;	
	quit = false;
	leftAnalogX = 0;
	leftAnalogY = 0;
	rightAnalogX = 0;
	rightAnalogY = 0;
	dPadX = 0;
	dPadY = 0;
	joystickButtons = 0;

	joystick_open();
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
#if 0
void joystickDrive::setMotor()
{
	readJoystick();
	
	double mag = abs(leftAnalogY);

	double realmag = sqrt(leftAnalogX*leftAnalogX + leftAnalogY*leftAnalogY) / (double(32768)*sqrt(2)) * 225;

	if(realmag < 50)
	{
		m_motorCtr.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);
		return;
	}

	double vel = mag / (double(32768)*sqrt(2)) * 110;

	double side = leftAnalogX / (double(32768)*sqrt(2)) * double(120) / double(5);

	double rvel = -1, lvel = -1;

	rvel = vel + side;
	//lvel = vel - 1.5*side + 55;
	lvel = vel - side;


	//turbo
	if((joystickButtons & 32) == 32)
	{
		rvel += 20;
		lvel += 20;
	}
	//turbo
	if((joystickButtons & 128) == 128)
	{
		rvel += 40;
		lvel += 40;
	}

	//force tight turn
	if((joystickButtons & 1) == 1)
	{
		m_motorCtr.setMotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, rvel);
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << 0 << " right: " << vel << std::endl;
		return;
	}
	else if((joystickButtons & 4) == 4)
	{
		m_motorCtr.setMotorPWM(MC_MOTOR_FORWARD, lvel, MC_MOTOR_FORWARD, 0);
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << vel << " right: " << 0 << std::endl;
		return;
	}
	double r = -1,l=-1;
	#if 0
	if(!qD.getEncoderVel(r,l))
	{
		std::cout << "r: " << r << "l: " << l << std::endl;
	}
	#endif
	std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << lvel << " right: " << rvel << " side: " << side <<std::endl;
	m_motorCtr.setMotorPWM(MC_MOTOR_FORWARD, rvel, MC_MOTOR_FORWARD, lvel);
}
#else
void joystickDrive::setMotor()
{
	readJoystick();
	char theta = -1*(getHeading() * 128/(M_PI) + 64 + 128);

	//double rmag = sqrt(rightAnalogX*rightAnalogX + rightAnalogY*rightAnalogY);
	//double lmag = sqrt(leftAnalogX*leftAnalogX + leftAnalogY*leftAnalogY);
	double rymag = -1*rightAnalogY;
	double lymag = -1*leftAnalogY;
	double norm_rymag = rymag / double(32768);
	double norm_lymag = lymag / double(32768);

	double mag = 100;
	int rvel = norm_rymag * double(mag);
	int lvel = norm_lymag * double(mag);
	
	if(abs(rvel) < 50 || isnan(rvel) || isinf(rvel) || rvel > 255)//dead zone
	{
		rvel = 0;
	}
	else
	{
		if(rvel > 0)
		{
			rvel = (rvel - 50) * 2;
		}
		else
		{
			rvel = (rvel + 50) * 2;
		}
	}
	if(abs(lvel) < 50 || isnan(lvel) || isinf(lvel) || lvel > 255)//dead zone
	{
		lvel = 0;
	}
	else
	{
		if(lvel > 0)
		{
			lvel = (lvel - 50) * 2;
		}
		else
		{
			lvel = (lvel + 50) * 2;
		}
	}
	//turbo
	if((joystickButtons & 32) == 32)
	{
		rvel += (rvel >= 0) ? 20 : -20;
		lvel += (lvel >= 0) ? 20 : -20;
	}
	//turbo
	if((joystickButtons & 128) == 128)
	{
		rvel += (rvel >= 0) ? 20 : -20;
		lvel += (lvel >= 0) ? 20 : -20;
	}

	//force tight turn
	if((joystickButtons & 1) == 1)
	{
		lvel = 0;
	}
	else if((joystickButtons & 4) == 4)
	{
		rvel = 0;
	}


	std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ") right at (" << rightAnalogX << "," << rightAnalogY << ")\twould have set lvel: " << lvel << " rvel: " << rvel << std::endl;


	//if(m_motorCtr->setMotorPWM(lvel, rvel,lvel, rvel))
	if(m_motorCtr->setMotorPWM(rvel, lvel,rvel, lvel))
	{
		std::cerr << "set failed" << std::endl;
	}
}
#endif
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
					//done = 1;
					//exit(-1);
					quit = true;
					break;
				default:
					break;
			}
		}
}

bool joystickDrive::shouldQuit()
{
	return(quit);
}

bool joystickDrive::manualOverride()
{
	return check_button(RT_TRIG);	
	// Checks if the manual override button is pressed
}

void joystickDrive::printLoop()
// Helper method for figuring out buttons on joystick
{
	for(;;)
	{
		readJoystick();	
		std::cout << "Button value: " << std::hex << joystickButtons << "\n";
		bool t = joystickButtons << 8;
		std::cout << t << "\n";	
		usleep(1e5);bool check_button(BUTTON);
	}
}

bool joystickDrive::check_button(BUTTON b)
{
	readJoystick();
	return (bool)(joystickButtons & 1<<b);
}

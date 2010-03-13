
#include "joystickDrive.hpp"

//static SDL_Joystick *joystick;

int main()
{
	joystickDrive jD;

	for(;;)
	{
		jD.setMotor();
		if(jD.shouldQuit())
		{
			return(0);
		}
		usleep(10000);
		
	}
}

joystickDrive::joystickDrive()
{
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
void joystickDrive::setMotor()
{
	readJoystick();
	
	double mag = sqrt(leftAnalogY*leftAnalogY);

	double realmag = sqrt(leftAnalogX*leftAnalogX + leftAnalogY*leftAnalogY) / (double(32768)*sqrt(2)) * 225;

	if(realmag < 50)
	{
		m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, 0);
		return;
	}

	double vel = mag / (double(32768)*sqrt(2)) * 160;

	double side = leftAnalogX / (double(32768)*sqrt(2)) * double(200) / double(5);

	double rvel = -1, lvel = -1;

	rvel = vel + side;
	lvel = vel - 1.5*side + 35;


	//turbo
	if((joystickButtons & 32) == 32)
	{
		rvel += 10;
		lvel += 10;
	}
	//turbo
	if((joystickButtons & 128) == 128)
	{
		rvel += 20;
		lvel += 20;
	}

	//force tight turn
	if((joystickButtons & 1) == 1)
	{
		m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, rvel);
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << 0 << " right: " << vel << std::endl;
		return;
	}
	else if((joystickButtons & 4) == 4)
	{
		m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, lvel, MC_MOTOR_FORWARD, 0);
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
	m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, rvel, MC_MOTOR_FORWARD, lvel);
}
#if 0
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
/*
	else if( theta > 120 || theta < -120 )//go backwards if the stick is nearly straight back
	{
		vel = scaleThrottle(vel);
		std::cout << "right at ("<< rightAnalogX << "," << rightAnalogY << std::endl;
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left:" << -1*vel << "right " << -1*vel << std::endl;
		m_motorCtr.set_motors(-1*vel, -1*vel);
		return;
	}
*/
	else
	{
		vel = scaleThrottle(vel);
	}

	//turbo
	if((joystickButtons & 32) == 32)
	{
		vel += 200;
	}
	//turbo
	if((joystickButtons & 128) == 128)
	{
		vel += 250;
	}

	//force tight turn
	if((joystickButtons & 1) == 1)
	{
		m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, 0, MC_MOTOR_FORWARD, vel);
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << 0 << " right: " << vel << std::endl;
		return;
	}
	else if((joystickButtons & 4) == 4)
	{
		m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, vel, MC_MOTOR_FORWARD, 0);
		std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set left: " << vel << " right: " << 0 << std::endl;
		return;
	}


	std::cout << "right at ("<< rightAnalogX << "," << rightAnalogY << ")\twould have set vel: " << vel << " head: " << (int)theta << std::endl;
	std::cout << "btn:" << joystickButtons << "\tleft at ("<< leftAnalogX << "," << leftAnalogY << ")\twould have set vel: " << vel << " head: " << (int)theta << std::endl;


	m_motorCtr.setmotorPWM(MC_MOTOR_FORWARD, vel+theta, MC_MOTOR_FORWARD, vel-theta);
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

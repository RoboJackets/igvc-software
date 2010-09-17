#include <SDL/SDL.h> //why can't gcc find this
//#include "/usr/include/SDL/SDL.h"
#include <pthread.h>
#include <errno.h>
#include <stdio.h>		// for printf in main()

#include "joystick.h"

volatile int leftAnalogX;
volatile int leftAnalogY;
volatile int rightAnalogX;
volatile int rightAnalogY;
volatile int dPadX;
volatile int dPadY;
volatile int joystickButtons;

static SDL_Joystick *joystick;

static void* joystick_run(void* noArg) {
	int i;
	int done;
	SDL_Event event;
	
	// Ignore initial events during startup
	for (i=0;i<20;i++) {
		SDL_PollEvent(&event);
	}
	
	// Poll the joystick for events, and update globals
	done = 0;
	while ( !done ) {
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
	
	// Close joystick if it is opened
	if (SDL_JoystickOpened(0)) {
		SDL_JoystickClose(joystick);
	}
	
	// Kill SDL
	SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
	
	return NULL;
}

static pthread_t joystick_thread;

/**
 * Opens the joystick.
 * 
 * @return		NULL if success;
 * 				or a message if error.
 */
const char* joystick_open(void) {
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
	err = pthread_create(
		&joystick_thread,
		NULL,
		&joystick_run,
		NULL);
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

/**
 * Closes the joystick.
 */
void joystick_close(void) {
	// Send an SDL_QUIT event to the joystick event loop
	SDL_QuitEvent quitEvent = { SDL_QUIT };
	SDL_PushEvent((SDL_Event*) &quitEvent);
}

// ---

int main(int argc, char *argv[]) {
	// Open joystick
	const char* err = joystick_open();
	if (err != NULL) {
		printf("Failed to initialize joystick: %s\n", err);
		return 1;
	}
	
	// Print joystick status every second
	for(;;) {
		printf("left=(%d,%d), right=(%d,%d), dpad=(%d,%d), btns=%d\n",
			leftAnalogX,
			leftAnalogY,
			rightAnalogX,
			rightAnalogY,
			dPadX,
			dPadY,
			joystickButtons);
		
		usleep(1000);	// 1 sec
	}
	
	// Close joystick
	joystick_close();
	
	return 0;
}


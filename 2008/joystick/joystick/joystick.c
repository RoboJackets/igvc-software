#include <SDL.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
	int x_move;
	int y_move;
	int garbage;
	int i;
	int done;
	int buttons = 0;
	int dPadX = 0;
	int dPadY = 0;
	int leftAnalogX = 0;
	int leftAnalogY = 0;
	int rightAnalogX = 0;
	int rightAnalogY = 0;
	SDL_Event event;
	SDL_Joystick *joystick;
	//setup starts:
	/* Initialize SDL */
	if ( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0 ) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
		exit(1);
	}
	//garbage = SDL_JoystickEventState(SDL_ENABLE);
	joystick = SDL_JoystickOpen(0);
	if (SDL_JoystickOpened(0)) {
		printf("Joystick Opened!\n");
	}
	else {
		printf("Failed to open joystick!\n");
	}
	done=0;
	SDL_PollEvent(&event);//clear out any bad data
	//setup ends
	//main loop starts:
	while ( ! done ) {
		while ( SDL_PollEvent(&event) ) {
			switch (event.type) {
			    case SDL_JOYAXISMOTION:
				//printf("Joystick %d axis %d value: %d\n",
				       //event.jaxis.which,
				       //event.jaxis.axis,
				       //event.jaxis.value);
				switch (event.jaxis.axis) {
					case 4:
						dPadX=event.jaxis.value;
						printf("dPadX: %d\n", dPadX);
					break;
					case 5:
						dPadY=event.jaxis.value;
						printf("dPadY: %d\n", dPadY);
					break;
					case 0:
						leftAnalogX=event.jaxis.value;
						printf("leftAnalogX: %d\n", leftAnalogX);
					break;
					case 1:
						leftAnalogY=event.jaxis.value;
						printf("leftAnalogY: %d\n", leftAnalogY);
					break;
					case 2:
						rightAnalogX=event.jaxis.value;
						printf("rightAnalogX: %d\n", rightAnalogX);
					break;
					case 3:
						rightAnalogY=event.jaxis.value;
						printf("rightAnalogY: %d\n", rightAnalogY);
					break;
					default:
					break;
				}
				break;
			    case SDL_JOYBUTTONDOWN:
				//printf("Joystick %d button %d down\n",
				       //event.jbutton.which,
				       //event.jbutton.button);
				       buttons = buttons | 1<<event.jbutton.button;
						printf("buttons: %d\n", buttons);
				break;
			    case SDL_JOYBUTTONUP:
				//printf("Joystick %d button %d up\n",
				       //event.jbutton.which,
				       //event.jbutton.button);
				       buttons = buttons & ~1<<event.jbutton.button;
						printf("buttons: %d\n", buttons);
				break;
			    case SDL_KEYDOWN:
				if ( event.key.keysym.sym != SDLK_ESCAPE ) {//this line doesnt work
					break;
				}
				/* Fall through to signal quit */
			    case SDL_QUIT:
				done = 1;
				break;
			    default:
				break;
			}
		}
	}
	//main loop ends
	
	//shutdown starts:
	  // Close if opened
	if(SDL_JoystickOpened(0)) {
		SDL_JoystickClose(joystick);
	}
	SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
	//shutdown ends
	return 0;
}







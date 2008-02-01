#include <SDL.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
	int x_move;
	int y_move;
	int garbage;
	int i;
	int done;
	SDL_Event event;
	SDL_Joystick *joystick;
	/* Initialize SDL */
	if ( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0 ) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
		exit(1);
	}
	//garbage = SDL_JoystickEventState(SDL_ENABLE);
	joystick = SDL_JoystickOpen(0);
	if (SDL_JoystickOpened(0)) {
		printf("JoystickOpened!\n");
	}
	else {
		printf("Fail: JoystickOpened\n");
	}

	for (i=0;i<100;i++) {
	printf("count: %d\n",i);
		done=0;
		while ( ! done ) {
			while ( SDL_PollEvent(&event) ) {
				switch (event.type) {
				    case SDL_JOYAXISMOTION:
					printf("Joystick %d axis %d value: %d\n",
					       event.jaxis.which,
					       event.jaxis.axis,
					       event.jaxis.value);
					break;
				    case SDL_KEYDOWN:
					if ( event.key.keysym.sym != SDLK_ESCAPE ) {
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
			x_move = (((int)SDL_JoystickGetAxis(joystick, 0))+32768);
			y_move = (((int)SDL_JoystickGetAxis(joystick, 1))+32768);
			//printf("x_move: %d\n",x_move);
			//printf("y_move: %d\n",y_move);
		}
		sleep(1);
		/*SDL_JoystickUpdate;
		x_move=SDL_JoystickGetAxis(joystick, 0);
		printf("x_move: %d\n",x_move);
		sleep(1);*/
	}











	  // Close if opened
	if(SDL_JoystickOpened(0))
		SDL_JoystickClose(joystick);
	SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
}







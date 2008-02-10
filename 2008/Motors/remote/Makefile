#$CC = gcc
#$(CC_FLAGS) = -Wall 
#joystick joystick.c

srcdir  = .

CC      = gcc
EXE	= 
CFLAGS  = -g -O2 -I/usr/local/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT -DHAVE_OPENGL
LIBS	=  -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lSDL -lpthread

all:

	gcc -o remote ./remote.c $(CFLAGS) $(LIBS)
	
clean:
	rm *.o

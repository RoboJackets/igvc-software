#Makefile for keyboardDrive.cc
#Paul Varnell

# pcg-config should give the correct includes and libraries on your
# system if you followed the installation instructions at
# http://playerstage.sourceforge.net/doc/Player-2.0.0/player/install.html
LDFLAGS += `pkg-config --libs playerc++`
CXXFLAGS += `pkg-config --cflags playerc++`		

# Position-independent code, not sure why needed, Matt Powers knows
CXXFLAGS += -fPIC

project_name = keyboardDrive

all:    $(project_name) 
clean:
	rm -rf $(project_name) *~ *.o

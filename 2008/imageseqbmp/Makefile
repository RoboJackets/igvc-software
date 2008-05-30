#
# Builds a Player plugin using libtool.
# 
# The plugin may include *.cc or *.c source files.
#
CC = gcc
CXX = g++
CXXLD = g++
LIBTOOL = libtool
PLAYER_CFLAGS = `pkg-config --cflags playercore`
INSTALL_DIR = /tmp
OUTPUT = libimageseqbmpplugin.la

all:
	$(LIBTOOL) --mode=compile $(CC) -c -g $(PLAYER_CFLAGS) *.c
	$(LIBTOOL) --mode=compile $(CXX) -c -g $(PLAYER_CFLAGS) *.cc
	$(LIBTOOL) --mode=link $(CXXLD) -rpath $(INSTALL_DIR) -module -export-dynamic -version-info 0:0:0 *.lo -o $(OUTPUT)

clean:
	rm -rf *.o *.lo *.la .libs

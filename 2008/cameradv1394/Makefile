#
# An example of how to build a Player plugin with libtool
#


CC = gcc
CXX = g++
CXXLD = g++
LIBTOOL = libtool
PLAYER_CFLAGS = `pkg-config --cflags playercore` #-I.
INSTALL_DIR = /tmp
OUTPUT = libcameraG1394plugin.la
LIBS = -ldc1394_control -lraw1394 

#all: libcameraG1394plugin.la

#libcameraG1394plugin.la: cameraG1394.lo
#	$(LIBTOOL) --mode=link $(CXXLD) $(LIBS) -rpath $(INSTALL_DIR) -module -export-dynamic -version-info 0:0:0 $< -o $@

#%.lo: %.cc
#	$(LIBTOOL) --mode=compile $(CXX) -c $(PLAYER_CFLAGS) $<


all:
	$(LIBTOOL) --mode=compile $(CC) -c $(PLAYER_CFLAGS) *.c
	$(LIBTOOL) --mode=compile $(CXX) -c $(PLAYER_CFLAGS) cameraG1394.cc
	$(LIBTOOL) --mode=link $(CXXLD) $(LIBS) -rpath $(INSTALL_DIR) -module -export-dynamic -version-info 0:0:0 *.lo -o $(OUTPUT)


clean:
	rm -rf *.o *.lo *.la .libs  

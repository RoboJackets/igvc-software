#
# An example of how to build a Player plugin with libtool
#

CXX = g++
CXXLD = g++
LIBTOOL = libtool
PLAYER_CFLAGS = `pkg-config --cflags playercore` -I.
LIBS = -ldc1394_control -lraw1394 -ldv
INSTALL_DIR = /tmp

all: libcameradv1394plugin.la

libcameradv1394plugin.la: cameradv1394.lo
	$(LIBTOOL) --mode=link $(CXXLD) $(LIBS) -rpath $(INSTALL_DIR) -module -export-dynamic -version-info 0:0:0 $< -o $@

%.lo: %.cc
	$(LIBTOOL) --mode=compile $(CXX) -c $(PLAYER_CFLAGS) $<

clean:
	rm -rf *.o *.lo *.la .libs  

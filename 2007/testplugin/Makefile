LIBS=

all:
	@# Create .o files
	libtool --mode=compile g++ -c `pkg-config --cflags playercore` -I/usr/include/libdv -I. *.cc
	
	libtool --mode=link g++ -shared `pkg-config --libs playercore` $(LIBS) *.lo -o libtestplugin.a
	libtool --mode=link g++ -shared `pkg-config --libs playercore` $(LIBS) *.lo -o libtestplugin.so
	libtool --mode=link g++ -shared `pkg-config --libs playercore` $(LIBS) *.lo -o libtestplugin.la
	
clean:
	rm -f *.o *.a *.dylib *.la *.lo *.so

# 09/4/2001 Modified by Paul for CYGWIN

.SUFFIXES: .c

#CFLAGS = -O2 -mno-cygwin # For Mingw
CFLAGS = getsurf.o transform.o play.o -O3 -lglut32 -lglu32 -lopengl32  -o transform.exe
LCFLAGS = getsurf.o transform.o play.o -L../vision -lvissys  -lGL -lglut   -o transform
INCLUDES = -I/usr/local/include/player-2.0/ -I/usr/include/qt4 -I/usr/include/qt4/QtCore

GLLIBS=-lglut32 -lglu32 -lopengl32 -I/usr/include/w32api/GL/ # Cygwin or Mingw
libs = ${GLLIBS} # Cygwin or Mingw

EXEEXT=.exe # Cygwin or Mingw

linux: vision
	g++ play.cpp -c $(INCLUDES)
	gcc transform.c -c 
	g++ $(LCFLAGS) `pkg-config --libs playercore playerc++` -lz -ldl -ljpeg -lQtCore
	#	-lqt-mt

cyg: 
	g++ play.cpp -c $(INCLUDES)
	gcc transform.c  -c
	g++ $(CFLAGS) `pkg-config --libs playercore playerc++` -lz -ldl -liconv -lrpclib -ljpeg -lQtCore

vision:
	make -C ../vision/
	
clean:
	rm -rf *.o transform
	
	

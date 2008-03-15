# 09/4/2001 Modified by Paul for CYGWIN

#.SUFFIXES: .c

#CFLAGS = -O2 -mno-cygwin # For Mingw
#CFLAGS =  -O3 -lglut32 -lglu32 -lopengl32 
LCFLAGS =  -L  -lGL -lglut   -o transform
CFLAGS = 
INCLUDES = 
#SOURCES=curtain.c texcode.c transform.c utilfuncs.c 
TARGETS=curtain.o texcode.o transform.o utilfuncs.o
#COMPILE  =curtain.c image.h Makefile Makefile~ test.bmp texcode.c texcode.c~ texcode.h transform.c transform.c~ transform.o utilfuncs.c 
OUTPUT_OPTION=-c

GLLIBS=-lglut32 -lglu32 -lopengl32 -I/usr/include/w32api/GL/ # Cygwin or Mingw
libs = ${GLLIBS} # Cygwin or Mingw

EXEEXT=.exe # Cygwin or Mingww

linux: $(TARGETS)
	make $(TARGETS)
	gcc transform.c -c 
	g++ $(TARGETS) $(LCFLAGS)

cyg: 
	g++ play.cpp -c $(INCLUDES)
	gcc transform.c  -c
	g++ $(CFLAGS) `pkg-config --libs playercore playerc++` -lz -ldl -liconv -lrpclib -ljpeg -lQtCore
	
clean:
	rm -rf *.o transform
	
	

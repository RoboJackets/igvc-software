# 09/4/2001 Modified by Paul for CYGWIN

#.SUFFIXES: .c

#CFLAGS = -O2 -mno-cygwin # For Mingw
#CFLAGS =  -O3 -lglut32 -lglu32 -lopengl32 
LCFLAGS =  -L  -lGL -lglut
CFLAGS = 
INCLUDES = 
LDLIBS = -L  -lGL -lglut
#SOURCES=curtain.c texcode.c transform.c utilfuncs.c 
#TARGETS=curtain.o texcode.o transform.o utilfuncs.o
TARGETS=`echo *.c|sed 's/\.c/\.o/g'` `echo *.cc|sed 's/\.cc/\.o/g'`
#COMPILE  =curtain.c image.h Makefile Makefile~ test.bmp texcode.c texcode.c~ texcode.h transform.c transform.c~ transform.o utilfuncs.c 
OUTPUT_OPTION=-c

GLLIBS=-lglut32 -lglu32 -lopengl32 -I/usr/include/w32api/GL/ # Cygwin or Mingw
libs = ${GLLIBS} # Cygwin or Mingw

EXEEXT=.exe # Cygwin or Mingww

all:
	make `echo *.c|sed 's/\.c/\.o/g'`
	make `echo *.cc|sed 's/\.cc/\.o/g'`
	g++ $(TARGETS) $(LCFLAGS) -o blackbarrels

linux: $(TARGETS)
	make $(TARGETS)
	gcc transform.c -c 
	g++ $(TARGETS) $(LCFLAGS)
	
clean:
	rm -rf *.o blackbarrels
	
	

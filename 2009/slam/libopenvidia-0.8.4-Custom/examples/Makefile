# directory where openvidia is.
ONVDIR = ../

# required flags for openvidia
ONV_FLAGS = -I$(ONVDIR)/include -L$(ONVDIR) -ldc1394_control -lraw1394  -lstdc++ -lccext2 -lccgnu2 -lxml  -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lCgGL -lpthread -lfltk  -lImlib
 
all: cgExample featureExample Dc1394example feature FLfeature V4L1example

clean:
	rm cgExample featureExample Dc1394example feature FLfeature V4L1example


#an example of how to compile with openvidia: 
#make sure your .cc is \before\ -lopenvidia

featureExample: featureExample.cc
	g++ featureExample.cc $(ONV_FLAGS) -O3  -lopenvidia -lm  -o featureExample

% : %.cc 
	g++ $< $(ONV_FLAGS) -O3 -lopenvidia -lm  -o $@

FLfeature : FLfeature.cc InterfaceWindow.cxx ../libopenvidia.a
	g++ $< $(ONV_FLAGS) InterfaceWindow.cxx -O3 -lopenvidia -lm -lfltk -lfltk_gl  `imlib2-config --libs` -o FLfeature 

# directory where openvidia is.
ONVDIR = ../

# required flags for openvidia
#ONV_FLAGS = -I/usr/include/opencv -I/usr/local/include/opencv -I. -I$(ONVDIR)/include -L$(ONVDIR) -L/usr/lib -L /usr/local/lib  -lcv -lcvaux -lhighgui -lstdc++ -lccext2 -lccgnu2   -lGL -lglut -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lCgGL -lpthread   -lImlib 
 
#all:  	feature 

#clean:	feature 

#an example of how to compile with openvidia: 
#make sure your .cc is \before\ -lopenvidia

#feature: feature.cc
#	g++ feature.cc $(ONV_FLAGS) -O3  -lopenvidia -lm  -o featureExample

#% : %.cc 
#	g++ $< $(ONV_FLAGS) -O3 -lopenvidia -lm  -o $@

######################################################################

TARGET := feature
OBJECTS := feature.o CVcam.o image_buffers.o vision.o
CXX := g++
CXXFLAGS := -I/usr/include/opencv -I/usr/local/include/opencv -I. -I$(ONVDIR)/include -L$(ONVDIR) -L/usr/lib -L/usr/local/lib -lltdl -lm -lGL -lglut -lcv -lcvaux -lhighgui -lstdc++  -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES -lCgGL -lpthread 

all:	$(TARGET)

clean:
	rm -rf $(TARGET) *.o

$(TARGET):	$(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -lopenvidia -lm -o $@

.cpp.o: 
	$(CXX) $(CXXFLAGS) -c $? -lopenvidia -lm -o $@
.cc.o:
	$(CXX) $(CXXFLAGS) -c $? -lopenvidia -lm -o $@

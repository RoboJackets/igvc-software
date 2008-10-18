TARGET := lasertest
OBJECTS := urg_laser.o urg_laser_test.o
CXX := g++
CXXFLAGS := -Wall

all:	$(TARGET)

clean:
	rm -rf $(TARGET) *.o

$(TARGET):	$(OBJECTS)
	$(CXX) $^ -o $@

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $? -o $@



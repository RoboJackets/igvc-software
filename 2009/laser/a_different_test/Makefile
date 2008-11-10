TARGET := lasertest
OBJECTS := urg_laser.o test.o
CXX := g++
CXXFLAGS := -Wall

all:	$(TARGET)

clean:
	rm -rf $(TARGET) *.o

$(TARGET):	$(OBJECTS)
	$(CXX) $^ -o $@

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $? -o $@



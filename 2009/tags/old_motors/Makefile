#g++ -o motorsTest `pkg-config --cflags playerc++` motorsTest.cc `pkg-config --libs playerc++` -I /usr/local/include/player-2.0/libplayerc++/

TARGET=motors_test

CXX = g++
PLAYER_CFLAGS = 

all: $(TARGET)

clean:
	rm -rf $(TARGET) *~ *.o *.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 $(PLAYER_CFLAGS) -c $<

$(TARGET).so: $(TARGET).o
	$(CXX) -shared -nostartfiles -o $@ $<

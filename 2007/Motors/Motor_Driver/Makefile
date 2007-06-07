#g++ -o motorsTest `pkg-config --cflags playerc++` motorsTest.cc `pkg-config --libs playerc++` -I /usr/local/include/player-2.0/libplayerc++/

TARGET=motors

CXX = g++
PLAYER_CFLAGS = `pkg-config --cflags playercore`

all: $(TARGET).so

clean:
	rm -rf $(TARGET) *~ *.o *.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 $(PLAYER_CFLAGS) -c $<

$(TARGET).so: $(TARGET).o
	$(CXX) -shared -nostartfiles -o $@ $<

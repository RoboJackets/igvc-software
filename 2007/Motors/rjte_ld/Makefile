TARGET=rjte_load
OBJS=rjte_load.o serial.o exception.o

SRCS=$(addsuffix .cc, $(basename $(OBJS)))

CXXFLAGS=-O2 -Wall -g3

all:		$(TARGET)

clean:
		rm -f $(TARGET) $(OBJS)

dep:
		makedepend $(CXXFLAGS) -- $(SRCS)

$(TARGET):	$(OBJS)
		$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)


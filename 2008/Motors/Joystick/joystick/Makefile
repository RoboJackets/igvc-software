TARGET	= joystick
CC      = gcc
CFLAGS  = -g -O2 -I/usr/local/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT -DHAVE_OPENGL
LDFLAGS	= -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lSDL -lpthread

all: $(TARGET)
	
clean:
	rm $(TARGET) *.o *.so

%.o: %.c
	$(CC) $(CFLAGS) -c $<

$(TARGET): $(TARGET).o
	$(CC) $(LDFLAGS) $^ -o $@

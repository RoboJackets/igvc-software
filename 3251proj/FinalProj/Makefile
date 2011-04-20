#################################################################
##
## FILE:	Makefile
## PROJECT:	CS 3251 Project 1 - Professor Traynor
## DESCRIPTION: Compile Project 1
##
#################################################################

CC=gcc

OS := $(shell uname -s)

# Extra LDFLAGS if Solaris
ifeq ($(OS), SunOS)
	LDFLAGS=-lsocket -lnsl
    endif

all: server 

client: client.c
	$(CC) client.c -o client

server: server.c
	$(CC) -pthread server.c -o gpsServer

clean:
	    rm -f client gpsServer *.o


all:
	g++  -c -DQT_THREAD_SUPPORT -lQtCore -I/usr/include/qt4 -I/usr/include/qt4/QtCore *.cc 
	ar rc libvissys.a *.o
	ranlib libvissys.a
	#ld *.o -r -o vissys.o

clean:
	rm -f *.o libvissys.a

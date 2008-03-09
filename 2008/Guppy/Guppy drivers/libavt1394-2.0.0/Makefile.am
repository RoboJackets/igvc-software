lib_LTLIBRARIES = libavt1394.la
libavt1394_la_SOURCES = avt1394.c avt1394.h
libavt1394_la_LDFLAGS = -version-info 2:0:0
bin_PROGRAMS = avtext
avtext_SOURCES = avt_stub.c
avtext_LDADD = -L. -L./.libs -L/usr/local/lib -L/usr/lib -lavt1394 -lraw1394 -ldc1394_control -lm
# headers to be installed
pkginclude_HEADERS = avt1394.h

AM_CPPFLAGS = -I$(top_srcdir) -Wall
lib_LTLIBRARIES =

pkgincludedir = $(includedir)/player-2.0/libplayerjpeg
pkginclude_HEADERS =

if INCLUDE_JPEG
lib_LTLIBRARIES += libplayerjpeg.la
pkginclude_HEADERS += playerjpeg.h
endif

libplayerjpeg_la_SOURCES = playerjpeg.h playerjpeg.c
libplayerjpeg_la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir)
libplayerjpeg_la_LIBADD = -ljpeg


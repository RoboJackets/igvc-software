AM_CPPFLAGS = -Wall -g -I$(top_srcdir)/client_libs -I$(top_srcdir)

if WITH_GSL
if INCLUDE_GL
if INCLUDE_GLU
if INCLUDE_GLUT
bin_PROGRAMS = pmaptest
lib_LTLIBRARIES = libpmap.la liblodo.la
endif
endif
endif
endif

#EXTRA_DIST = pmap.dox

libpmap_la_SOURCES = logfile.cpp omap.cpp pmap.cpp rmap.cpp slap.cpp \
                     lodo.h omap.h pmap.h rmap.h slap.h logfile.h
libpmap_la_CFLAGS = @GSL_CFLAGS@ --fast-math

liblodo_la_SOURCES = lodo.cpp lodo.h
liblodo_la_CFLAGS = @GSL_CFLAGS@ --fast-math
liblodo_la_LIBADD = libpmap.la $(top_builddir)/libplayercore/libplayererror.la $(top_builddir)/client_libs/libplayerc/libplayerc.la $(top_builddir)/libplayerxdr/libplayerxdr.la

pmaptest_SOURCES = pmap_test.cpp lodo.h pmap.h rmap.h omap.h
pmaptest_LDADD = @GSL_LIBS@ -lpmap -llodo

doc:
	doxygen pmap.dox


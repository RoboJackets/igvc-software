AM_CPPFLAGS = -g -fPIC -Wall -I$(top_srcdir)

# create the pkg-config entry for the server headers
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = playertcp.pc

EXTRA_DIST = playertcp.pc.in
CLEANFILES = playertcp.pc

lib_LTLIBRARIES = libplayertcp.la

libplayertcp_la_DEPENDENCIES = $(top_builddir)/libplayercore/libplayercore.la $(top_builddir)/libplayercore/libplayererror.la $(top_builddir)/libplayerxdr/libplayerxdr.la

libplayertcp_la_SOURCES = socket_util.c socket_util.h \
                         playertcp.cc playertcp.h \
			 remote_driver.cc remote_driver.h

libplayertcp_la_LIBADD = $(top_builddir)/replace/*.lo
libplayertcp_la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir) $(top_builddir)/libplayercore/libplayercore.la $(top_builddir)/libplayercore/libplayererror.la $(top_builddir)/libplayerxdr/libplayerxdr.la

pkgincludedir = $(includedir)/player-2.0/libplayertcp

pkginclude_HEADERS = playertcp.h

AM_CPPFLAGS = -g -fPIC -Wall -D PLAYER_INSTALL_PREFIX="\"@PREFIX@\"" -I$(top_srcdir)

# create the pkg-config entry for the server headers
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = playercore.pc playererror.pc

EXTRA_DIST = playercore.pc.in playererror.pc.in

BUILT_SOURCES = playerconfig.h
CLEANFILES = playerconfig.h playercore.pc playererror.pc

lib_LTLIBRARIES = libplayererror.la libplayercore.la

libplayercore_la_SOURCES = driver.cc driver.h \
                          device.cc device.h \
                          drivertable.h drivertable.cc \
                          devicetable.h devicetable.cc \
                          configfile.cc configfile.h \
                          playercommon.h player.h playerconfig.h.in \
                          message.h message.cc \
                          wallclocktime.cc wallclocktime.h playertime.h \
                          interface_util.cc interface_util.h \
                          plugins.cc plugins.h \
                          globals.cc globals.h \
                          addr_util.c addr_util.h
libplayercore_la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir) $(top_builddir)/libplayercore/libplayererror.la
libplayercore_la_DEPENDENCIES = $(top_builddir)/libplayercore/libplayererror.la

libplayererror_la_SOURCES = error.h error.c
libplayererror_la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir)

coreincludedir = $(includedir)/player-2.0/libplayercore
coreinclude_HEADERS = configfile.h \
                      device.h \
                      devicetable.h \
                      driver.h \
                      drivertable.h \
                      globals.h \
                      interface_util.h \
                      message.h \
                      player.h \
                      playercommon.h \
                      playerconfig.h \
                      playercore.h \
                      playertime.h \
                      plugins.h \
                      wallclocktime.h \
                      error.h \
                      addr_util.h


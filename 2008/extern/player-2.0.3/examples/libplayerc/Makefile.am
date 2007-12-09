PLAYERC_CFLAGS = -I$(top_srcdir) \
                 -I$(top_srcdir)/client_libs/

PLAYERC_LIB_DIRS=-L${top_builddir}/libplayerxdr \
                 -L${top_builddir}/libplayercore \
                 -L${top_builddir}/client_libs/libplayerc

PLAYERC_DEPS = $(top_builddir)/client_libs/libplayerc/libplayerc.la \
               $(top_builddir)/libplayerxdr/libplayerxdr.la \
               $(top_builddir)/libplayercore/libplayererror.la

AM_CPPFLAGS = $(PLAYERC_CFLAGS) -Wall -g

exampledir = $(prefix)/share/player/examples/libplayerc

example_PROGRAMS = simple speech_c_client
example_DATA = simple.c speech_c_client.c

simple_SOURCES = simple.c
simple_LDADD = -lplayerc $(PLAYERC_LIB_DIRS) -lm -lplayerxdr -lplayererror
simple_DEPENDENCIES = $(PLAYERC_DEPS)

speech_c_client_SOURCES = speech_c_client.c
speech_c_client_LDADD = -lplayerc $(PLAYERC_LIB_DIRS) -lm -lplayerxdr -lplayererror
speech_c_client_DEPENDENCIES = $(PLAYERC_DEPS)

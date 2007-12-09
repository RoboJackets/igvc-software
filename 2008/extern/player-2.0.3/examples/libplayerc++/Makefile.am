PLAYERCC_CFLAGS = -I$(top_srcdir) \
                  -I$(top_srcdir)/client_libs/ \
                  $(PLAYERCC_EXTRA_CFLAGS)

PLAYERCC_LIB_DIRS=-L${top_builddir}/libplayerxdr \
                  -L${top_builddir}/libplayercore \
                  -L${top_builddir}/client_libs/libplayerc \
                  -L${top_builddir}/client_libs/libplayerc++

PLAYERCC_DEPS = $(top_builddir)/client_libs/libplayerc/libplayerc.la \
                $(top_builddir)/libplayerxdr/libplayerxdr.la \
                $(top_builddir)/libplayercore/libplayererror.la \
                $(top_builddir)/client_libs/libplayerc++/libplayerc++.la

AM_CPPFLAGS = $(PLAYERCC_CFLAGS) -Wall -g

exampledir = $(prefix)/share/player/examples/libplayerc++

example_PROGRAMS =

if INCLUDE_PLAYERCC
  example_PROGRAMS += camera \
                     example0 \
                     example4 \
                     grip \
                     ptz \
                     laserobstacleavoid \
                     randomwalk \
                     clientgraphics \
                     sonarobstacleavoid \
                     speech \
                     wallfollow
endif

if INCLUDE_PLAYERCC_BOOST
  example_PROGRAMS += example1 example3 goto speech_cpp_client
endif

if INCLUDE_BOOST_SIGNALS
  example_PROGRAMS += example2
endif

example_DATA = args.h \
               camera.cc \
               clientgraphics.cc \
               example0.cc \
               example1.cc \
               example2.cc \
               example3.cc \
               example4.cc \
               goto.cc \
               grip.cc \
               laserobstacleavoid.cc \
               ptz.cc \
               randomwalk.cc \
               sonarobstacleavoid.cc \
               speech.cc \
               speech_cpp_client.cc \
               wallfollow.cc

camera_SOURCES = camera.cc args.h
camera_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
camera_DEPENDENCIES = $(PLAYERCC_DEPS)

example0_SOURCES = example0.cc
example0_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
example0_DEPENDENCIES = $(PLAYERCC_DEPS)

example1_SOURCES = example1.cc args.h
example1_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
example1_DEPENDENCIES = $(PLAYERCC_DEPS)

example2_SOURCES = example2.cc
example2_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
example2_DEPENDENCIES = $(PLAYERCC_DEPS)

example3_SOURCES = example3.cc args.h
example3_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
example3_DEPENDENCIES = $(PLAYERCC_DEPS)

example4_SOURCES = example4.cc
example4_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
example4_DEPENDENCIES = $(PLAYERCC_DEPS)

goto_SOURCES = goto.cc
goto_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
goto_DEPENDENCIES = $(PLAYERCC_DEPS)

grip_SOURCES = grip.cc args.h
grip_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
grip_DEPENDENCIES = $(PLAYERCC_DEPS)

clientgraphics_SOURCES = clientgraphics.cc args.h
clientgraphics_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
clientgraphics_DEPENDENCIES = $(PLAYERCC_DEPS)

laserobstacleavoid_SOURCES = laserobstacleavoid.cc args.h
laserobstacleavoid_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
laserobstacleavoid_DEPENDENCIES = $(PLAYERCC_DEPS)

ptz_SOURCES = ptz.cc args.h
ptz_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
ptz_DEPENDENCIES = $(PLAYERCC_DEPS)

randomwalk_SOURCES = randomwalk.cc args.h
randomwalk_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
randomwalk_DEPENDENCIES = $(PLAYERCC_DEPS)

sonarobstacleavoid_SOURCES = sonarobstacleavoid.cc args.h
sonarobstacleavoid_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
sonarobstacleavoid_DEPENDENCIES = $(PLAYERCC_DEPS)

speech_SOURCES = speech.cc args.h
speech_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
speech_DEPENDENCIES = $(PLAYERCC_DEPS)

speech_cpp_client_SOURCES = speech_cpp_client.cc
speech_cpp_client_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
speech_cpp_client_DEPENDENCIES = $(PLAYERCC_DEPS)

wallfollow_SOURCES = wallfollow.cc args.h
wallfollow_LDADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB) -lplayerc++ -lplayerxdr -lplayererror -lplayerc
wallfollow_DEPENDENCIES = $(PLAYERCC_DEPS)

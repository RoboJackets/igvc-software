if WITH_TESTS
BUILDTEST = test
else
BUILDTEST =
endif

AM_CPPFLAGS = -I$(srcdir)/.. -I$(top_srcdir) -g -Wall
test_LDADD = -L$(top_builddir)/client_libs/libplayerc -lplayerc -L$(top_builddir)/libplayerxdr -lplayerxdr -L$(top_builddir)/libplayercore -lplayererror -lm
#-L$(top_builddir)/client_libs/libplayerpacket -lplayerpacket
EXTRA_DIST = test.world test.cfg cave.pnm

noinst_PROGRAMS = $(BUILDTEST)

test_SOURCES = \
	test.c \
	test.h \
	test_blobfinder.c \
	test_camera.c \
	test_graphics2d.c \
	test_gripper.c \
	test_laser.c \
	test_log.c \
	test_map.c \
	test_position2d.c \
	test_position3d.c \
	test_power.c \
	test_ptz.c \
	test_simulation.c \
	test_sonar.c \
	test_rfid.c \
	test_wsn.c

#test_camera.c \
#test_fiducial.c \
#test_gps.c \
#test_joystick.c \
#test_localize.c \
#test_motor.c \
#test_truth.c \
#test_wifi.c

clean-local:
	rm -f test

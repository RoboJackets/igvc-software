if WITH_GTK
PLAYERV = playerv
endif

EXTRA_DIST = playerv-laser-blobfinder-ptz.jpg playerv-sonar.jpg

AM_CPPFLAGS = -I$(top_srcdir) -I$(top_srcdir)/client_libs @GTK_CFLAGS@

bin_PROGRAMS = $(PLAYERV)

playerv_SOURCES = playerv.c \
                  playerv.h \
                  registry.c \
                  mainwnd.c \
                  error.h \
                  opt.c \
                  opt.h \
		  dev_aio.c \
                  dev_blobfinder.c \
                  dev_bumper.c \
                  dev_dio.c \
                  dev_fiducial.c \
                  dev_ir.c \
                  dev_laser.c \
                  dev_map.c \
                  dev_position2d.c \
                  dev_power.c \
                  dev_ptz.c \
                  dev_sonar.c \
                  dev_wifi.c \
                  dev_gripper.c
#dev_localize.c

playerv_DEPENDENCIES = $(top_builddir)/client_libs/libplayerc/libplayerc.la \
                       $(top_builddir)/libplayerxdr/libplayerxdr.la \
                       $(top_builddir)/libplayercore/libplayererror.la
playerv_LDADD = $(top_builddir)/client_libs/libplayerc/libplayerc.la $(top_builddir)/libplayerxdr/libplayerxdr.la $(top_builddir)/libplayercore/libplayererror.la -lrtk -L$(top_builddir)/rtk2 -lrtk @GTK_LIBS@

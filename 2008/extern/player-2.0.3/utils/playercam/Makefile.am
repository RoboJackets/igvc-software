AM_CPPFLAGS = -I$(top_srcdir) -I$(top_srcdir)/client_libs @GTK_CFLAGS@

bin_PROGRAMS =

if WITH_GTK
bin_PROGRAMS += playercam
endif

playercam_SOURCES = playercam.c
playercam_LDADD = $(top_builddir)/client_libs/libplayerc/libplayerc.la $(top_builddir)/libplayerxdr/libplayerxdr.la $(top_builddir)/libplayercore/libplayererror.la @GTK_LIBS@

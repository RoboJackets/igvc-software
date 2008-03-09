# the libdc1394_control itself
MAINTAINERCLEANFILES = Makefile.in
lib_LTLIBRARIES = libdc1394_control.la

libdc1394_control_la_LDFLAGS = -version-info @lt_major@:@lt_revision@:@lt_age@

libdc1394_control_la_SOURCES =  \
	dc1394_control.c 	\
	dc1394_capture.c	\
        dc1394_control.h	\
	dc1394_format7.c        \
	dc1394_absolute.c	\
	dc1394_internal.h       \
	kernel-video1394.h

# headers to be installed
pkginclude_HEADERS = dc1394_control.h


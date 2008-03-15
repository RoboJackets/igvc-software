MAINTAINERCLEANFILES = Makefile.in

# the libraw1394 itself

lib_LTLIBRARIES = libraw1394.la

libraw1394_la_LDFLAGS = -version-info @lt_major@:@lt_revision@:@lt_age@

libraw1394_la_SOURCES =   \
	main.c            \
	eventloop.c       \
	errors.c          \
	readwrite.c       \
	iso.c             \
	fcp.c             \
	arm.c             \
	version.c         \
	kernel-raw1394.h  \
	raw1394_private.h \
	ieee1394-ioctl.h

# headers to be installed
pkginclude_HEADERS = raw1394.h csr.h ieee1394.h

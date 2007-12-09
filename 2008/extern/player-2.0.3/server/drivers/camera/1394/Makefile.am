AM_CPPFLAGS = -Wall -I$(top_srcdir)  -I$(srcdir)/../../blobfinder/cmvision

noinst_LTLIBRARIES =
if INCLUDE_CAMERA1394
noinst_LTLIBRARIES += libcamera1394.la
endif

libcamera1394_la_SOURCES = camera1394.cc


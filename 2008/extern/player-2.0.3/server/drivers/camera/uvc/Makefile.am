AM_CPPFLAGS = -Wall -I$(top_srcdir)

noinst_LTLIBRARIES =
if INCLUDE_CAMERAUVC
noinst_LTLIBRARIES += libcamerauvc.la
endif

libcamerauvc_la_SOURCES = cameraUVC.cc utils.h v4l2uvc.h utils.c v4l2uvc.c


noinst_LTLIBRARIES = 
if INCLUDE_WAVEFRONT
noinst_LTLIBRARIES += libwavefront.la
endif

AM_CPPFLAGS = -Wall -g -I$(top_srcdir)

libwavefront_la_SOURCES = plan.c plan_plan.c plan_waypoint.c plan.h \
                          wavefront.cc 
#heap.h heap.c

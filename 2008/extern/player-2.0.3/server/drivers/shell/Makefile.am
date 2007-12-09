
noinst_LTLIBRARIES = 
if INCLUDE_LOGFILE
noinst_LTLIBRARIES += liblogfile.la
endif
if INCLUDE_DUMMY
noinst_LTLIBRARIES += libdummy.la
endif
if INCLUDE_RELAY
noinst_LTLIBRARIES += librelay.la
endif

AM_CPPFLAGS = -g3 -Wall -I$(top_srcdir)

libdummy_la_SOURCES = dummy.cc
#libpassthrough_a_SOURCES = passthrough.cc 
liblogfile_la_SOURCES = writelog.cc encode.h encode.cc \
                        readlog_time.h readlog_time.cc readlog.cc 
librelay_la_SOURCES = relay.cc 



liblogfile_la_LDFLAGS = 
if HAVE_LIBZ
liblogfile_la_LDFLAGS += -lz
endif

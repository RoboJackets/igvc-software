noinst_LTLIBRARIES = 
if INCLUDE_SICKLMS200
noinst_LTLIBRARIES += libsicklms200.la
endif
if INCLUDE_SICKS3000
noinst_LTLIBRARIES += libsicks3000.la
endif
if INCLUDE_LASERCSPACE
noinst_LTLIBRARIES += liblasercspace.la
endif
if INCLUDE_SICKPLS
noinst_LTLIBRARIES += libsickpls.la
endif
if INCLUDE_LASERPOSEINTERPOLATOR
noinst_LTLIBRARIES += liblaserposeinterpolator.la
endif
if INCLUDE_LASERRESCAN
noinst_LTLIBRARIES += liblaserrescan.la
endif
if INCLUDE_URGLASER
noinst_LTLIBRARIES += liburglaser.la
endif

EXTRA_DIST = lasercspace-1.jpg lasercspace-2.jpg

AM_CPPFLAGS = -Wall -I$(top_srcdir)

libsicklms200_la_SOURCES = sicklms200.cc
libsicks3000_la_SOURCES = sicks3000.cc
libsickpls_la_SOURCES = sickpls.cc
liblaserposeinterpolator_la_SOURCES = laserposeinterpolator.cc
liblasercspace_la_SOURCES = lasercspace.cc lasertransform.h lasertransform.cc
liblaserrescan_la_SOURCES = laserrescan.cc lasertransform.h lasertransform.cc
liburglaser_la_SOURCES = urg_laser.cc urg_laser.h urglaserdriver.cc

AM_CPPFLAGS = -Wall -I$(top_srcdir)/server

noinst_LIBRARIES = @NOMAD_LIB@ 

EXTRA_LIBRARIES = libnomad.a

# CHOOSE A NOMADICS INTERFACE
#
# for connection via Nserver simulator/visualization (good for debugging)
NOMADICS_INTERFACE = Nclient.c

# for direct connection to a robot
#NOMADICS_INTERFACE = direct.c

libnomad_a_SOURCES = \
  Nclient.h \
  ${NOMADICS_INTERFACE} \
  nomad.cc \
  nomad_position.cc \
  nomad_sonar.cc

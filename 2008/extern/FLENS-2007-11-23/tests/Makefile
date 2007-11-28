ifndef FLENS_HOME
FLENS_HOME = $(PWD)/..
endif

include ../Makefile.common

SOURCEFILES  = $(wildcard *.cc)
TARGETS      = $(patsubst %.cc,%,$(SOURCEFILES))
LINK         = -L.. -lflens $(LDFLAGS) $(EX_LDFLAGS)

all: $(TARGETS)

clean:
	$(RM) *.o
	$(RM) $(TARGETS)
	
$(TARGETS): % : %.cc
	$(CXX) $(CXXFLAGS) $(INCDIRS) -I . -o $@ $< $(LINK)

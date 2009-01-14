ONV_INCLUDES =  -I./include 

LIB1394_LIBS = -ldc1394_control -lraw1394

COMMONCPP_LIBS = -lstdc++ -lccext2 -lccgnu2 -lxml

OPENGL_INCLUDES = -DGL_GLEXT_PROTOTYPES -DGLX_GLXEXT_PROTOTYPES
OPENGL_LIBS = -lCg -lpthread  -lCgGL -lglut

INCLUDES = $(ONV_INCLUDES) $(OPENGL_INCLUDES) 
LIBS    = $(LIB1394_LIBS) $(COMMONCPP_LIBS) $(OPENGL_LIBS)

CC = g++  -Wall -O3
OUTDIR = objs
SRCDIR = src

CFLAGS = -DUNIX -O3  -msse -minline-all-stringops\
        -mpush-args   \
        -falign-functions=4 -funroll-loops \
        -fexpensive-optimizations -fschedule-insns2


VO_OBJS = \
	$(OUTDIR)/mat_util.o\
	$(OUTDIR)/corners2r.o\
	$(OUTDIR)/corners2d.o\
	$(OUTDIR)/motion.o \
	$(OUTDIR)/pinverse.o
 
	
LIB_OBJS = \
	$(OUTDIR)/Dc1394.o\
	$(OUTDIR)/V4L1.o\
	$(OUTDIR)/conversions.o\
    $(OUTDIR)/PCT.o\
    $(OUTDIR)/fvutil.o\
	$(OUTDIR)/featureTrack.o\
	$(OUTDIR)/Scene.o\
	$(OUTDIR)/FBO_Filter.o\
    $(VO_OBJS)


libopenvidia.a : include/openvidia/tnt $(LIB_OBJS) $(SRCDIR)/*.h include/openvidia/*.h
	ar crs libopenvidia.a $(LIB_OBJS)
	ranlib libopenvidia.a

$(OUTDIR)/%.o : $(SRCDIR)/%.cc $(SRCDIR)/*.h
	mkdir -p $(OUTDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OUTDIR)/*.o
	rm -f libopenvidia.a
	rm -rf doc/html doc/latex doc/openvidia

documentation:
	doxygen doc/doxygenconfig

dist-clean:
	rm src/GPUHT.cc src/Hilbert.cc src/Hough.cc examples/cannyhough.cc examples/VP-basic.cg examples/FP-houghVis.cg


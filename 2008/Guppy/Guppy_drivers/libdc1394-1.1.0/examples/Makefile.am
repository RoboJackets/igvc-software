MAINTAINERCLEANFILES = Makefile.in
bin_PROGRAMS = dc1394_vloopback
noinst_PROGRAMS = grab_gray_image grab_partial_image dc1394_multiview grab_color_image

grab_gray_image_SOURCES = grab_gray_image.c
grab_gray_image_LDADD = ../libdc1394/libdc1394_control.la -lraw1394

grab_color_image_SOURCES = grab_color_image.c
grab_color_image_LDADD = ../libdc1394/libdc1394_control.la -lraw1394

grab_partial_image_SOURCES = grab_partial_image.c
grab_partial_image_LDADD = ../libdc1394/libdc1394_control.la -lraw1394

dc1394_multiview_CFLAGS = $(X_CFLAGS)
dc1394_multiview_SOURCES = dc1394_multiview.c
dc1394_multiview_LDADD = ../libdc1394/libdc1394_control.la -lraw1394 $(X_LIBS) $(X_PRE_LIBS) -lXv -lXext -lX11 $(X_EXTRA_LIBS)

dc1394_vloopback_SOURCES = dc1394_vloopback.c affine.c affine.h
dc1394_vloopback_LDADD = -lm ../libdc1394/libdc1394_control.la -lraw1394

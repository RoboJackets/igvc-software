
dnl Here are the tests for inclusion of Player's various device drivers

PLAYER_DRIVERS=
PLAYER_DRIVER_EXTRA_LIBS=
PLAYER_NODRIVERS=

dnl This macro can be used to setup the testing and associated autoconf
dnl variables and C defines for a device driver.
dnl
dnl PLAYER_ADD_DRIVER(name,default,[header],[cppadd],[ldadd],
dnl                   [pkgvar],[pkg],[default-includes])
dnl
dnl Args:
dnl   name:    name
dnl   default: should this driver be included by default? ("yes" or "no")
dnl   header:  a list of headers that are all required to build the driver
dnl   cppadd:  compiler flags to be used when building the driver
dnl            (e.g., "-I/somewhere_odd/include")
dnl   ldadd:   link flags to be added to Player if this driver is included
dnl            (e.g., "-lgsl -lcblas")
dnl   pkgvar:  variable prefix to be used with pkg-config; if your package
dnl            is found, pkgvar_CFLAGS and pkgvar_LIBS will be set
dnl            appropriately
dnl   pkg:     name of package that is required to build the driver; it
dnl            should be a valid pkg-config spec, like [gtk+-2.0 >= 2.1]
dnl   default-includes: A chunk of code to insert in a test program when
dnl                     checking for each of the headers listed in <header>.
dnl                     For example, if one of the headers that you need in
dnl                     turn requires <foo.h>, then supply
dnl                     [[#include <foo.h>]] here.
dnl
dnl The C define INCLUDE_<name> and the automake conditional <name> (with
dnl <name> capitalized) will be conditionally defined to be 1 and
dnl TRUE, respectively.  The variable <name>_EXTRA_CPPFLAGS will be
dnl the value of <cppadd>, for use in the driver's Makefile.am.
dnl
AC_DEFUN([PLAYER_ADD_DRIVER], [
AC_DEFUN([name_caps],translit($1,[a-z],[A-Z]))

user_override=no
ifelse($2,[yes],
  [AC_ARG_ENABLE($1,[  --disable-$1       Don't compile the $1 driver],user_override=yes,enable_$1=yes)],
  [AC_ARG_ENABLE($1, [  --enable-$1       Compile the $1 driver],user_override=yes,enable_$1=no)])

if test "x$enable_alldrivers" = "xno" -a "x$user_override" = "xno"; then
  enable_$1=no
fi

failed_header_check=no
failed_package_check=no
no_pkg_config=no
if test "x$enable_$1" = "xyes" -a len($3) -gt 0; then
dnl This little bit of hackery keeps us from generating invalid shell code,
dnl in the form of 'for' over an empty list.
  if test len($3) -gt 0; then
    header_list=$3
  else
    header_list=foo
  fi
  for header in $header_list; do
    AC_CHECK_HEADER($header,
                    enable_$1=yes,
                    enable_$1=no
                    failed_header_check=yes,$8)
  done
  if test "x$failed_header_check" = "xyes"; then
    enable_$1=no
  fi
fi

if test "x$enable_$1" = "xyes" -a len($6) -gt 0 -a len($7) -gt 0; then
  if test "x$have_pkg_config" = "xyes" ; then
    PKG_CHECK_MODULES($6, $7,
      enable_$1=yes,
      enable_$1=no
      failed_package_check=yes)
  else
    no_pkg_config=yes
    enable_$1=no
  fi
fi

if test "x$enable_$1" = "xyes"; then
  AC_DEFINE([INCLUDE_]name_caps, 1, [include the $1 driver])
  name_caps[_LIB]=[lib]$1[.la]
  name_caps[_EXTRA_CPPFLAGS]=$4
  name_caps[_EXTRA_LIB]=$5
  PLAYER_DRIVERS="$PLAYER_DRIVERS $1"
else
  if test "x$no_pkg_config" = "xyes"; then
    PLAYER_NODRIVERS="$PLAYER_NODRIVERS:$1 -- pkg-config is required to test for dependencies"
  elif test "x$failed_package_check" = "xyes"; then
    PLAYER_NODRIVERS="$PLAYER_NODRIVERS:$1 -- couldn't find required package $7"
  elif test "x$failed_header_check" = "xyes"; then
    PLAYER_NODRIVERS="$PLAYER_NODRIVERS:$1 -- couldn't find (at least one of) $header_list"
  elif test "x$2" = "xno"; then
    PLAYER_NODRIVERS="$PLAYER_NODRIVERS:$1 -- disabled by default; use --enable-$1 to enable"
  else
    PLAYER_NODRIVERS="$PLAYER_NODRIVERS:$1 -- disabled by user"
  fi
fi

AC_SUBST(name_caps[_LIB])
AM_CONDITIONAL([INCLUDE_]name_caps, test "x$enable_$1" = "xyes")
AC_SUBST(name_caps[_EXTRA_CPPFLAGS])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $name_caps[_EXTRA_LIB]"

])

AC_DEFUN([PLAYER_DRIVERTESTS], [

AC_ARG_ENABLE(playerclient_thread, [  --disable-playerclient_thread   Build thread safe c++ client library],,disable_playerclient_thread=no)
if test "x$disable_playerclient_thread" = "xno"; then
  AC_DEFINE(PLAYERCLIENT_THREAD,1,[Thread Safe C++ Client Library])
fi



PLAYER_ADD_DRIVER([acoustics],[no],
                  ["gsl/gsl_fft_real.h sys/soundcard.h"],[],
                  ["-lgsl -lgslcblas"])

PLAYER_ADD_DRIVER([acts],[yes],[],[],[])

dnl TODO: handle pkg-config location of gsl.  Some, but not all,
dnl installation of gsl have a .pc file.
dnl PKG_CHECK_MODULES(GSL,gsl,
dnl                   found_gsl=yes,
dnl                  found_gsl=no)
PLAYER_ADD_DRIVER([amcl], [yes],[gsl/gsl_version.h],[],["-lgsl -lgslcblas"])

PLAYER_ADD_DRIVER([amtecpowercube],[yes],[],[],[])

PLAYER_ADD_DRIVER([artoolkitplus],[yes],[],[],[],[ARTOOLKITPLUS],[artoolkitplus >= 2.0.2])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $ARTOOLKITPLUS_LIBS"

PLAYER_ADD_DRIVER([aodv],[yes],[],[],[])

PLAYER_ADD_DRIVER([bumpersafe],[yes],[],[],[])

dnl Check to see if we have version 1 or 2 API for dc1394
AC_CHECK_HEADER(dc1394/dc1394_control.h,[PLAYER_ADD_DRIVER([camera1394],[yes],["libraw1394/raw1394.h dc1394/dc1394_control.h"],[],["-lraw1394 -ldc1394"])],
	[PLAYER_ADD_DRIVER([camera1394],[yes],["libraw1394/raw1394.h libdc1394/dc1394_control.h"],[],["-lraw1394 -ldc1394_control"])])

dnl libdc1394 has varying API's, depending on the version.  Do some checks
dnl to see what the function signatures look like
if test "x$enable_camera1394" = "xyes"; then

  dc1394_dma_setup_args="0"

  AC_COMPILE_IFELSE(AC_LANG_PROGRAM(
    [[#include "libdc1394/dc1394_control.h"]],
    [[dc1394_dma_setup_capture(NULL, 0, 0, 0, 0, 0, 0, 0, 0, NULL, NULL)]]),
    dc1394_dma_setup_args="11")

  AC_COMPILE_IFELSE(AC_LANG_PROGRAM(
    [[#include "libdc1394/dc1394_control.h"]],
    [[dc1394_dma_setup_capture(NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, NULL, NULL)]]),
    dc1394_dma_setup_args="12")

  AC_COMPILE_IFELSE(AC_LANG_PROGRAM(
    [[#include "dc1394/dc1394_control.h"]],
    []),
    dc1394_dma_setup_args="20")

  AC_DEFINE_UNQUOTED(DC1394_DMA_SETUP_CAPTURE_ARGS, $dc1394_dma_setup_args,
              [arg count for dma capture function])
fi

PLAYER_ADD_DRIVER([cameracompress],[yes],[jpeglib.h],[],[-ljpeg])

PLAYER_ADD_DRIVER([camerauvc],[yes],[linux/videodev2.h],[],[])

PLAYER_ADD_DRIVER([camerav4l],[yes],[linux/videodev.h],[],[])

PLAYER_ADD_DRIVER([canonvcc4],[yes],[],[],[])

PLAYER_ADD_DRIVER([clodbuster],[yes],[],[],[])

PLAYER_ADD_DRIVER([cmucam2],[yes],[],[],[])

PLAYER_ADD_DRIVER([cmvision],[yes],[],[],[])

PLAYER_ADD_DRIVER([dummy],[yes],[],[],[])

PLAYER_ADD_DRIVER([er1],[yes],[asm/ioctls.h],[],[])

PLAYER_ADD_DRIVER([fakelocalize], [yes],[],[],[])

PLAYER_ADD_DRIVER([festival],[yes],[],[],[])

PLAYER_ADD_DRIVER([flockofbirds],[yes],[],[],[])

PLAYER_ADD_DRIVER([garcia],[no],
                  [],[],[],[GARCIA],[libgarcia])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $GARCIA_LIBS"

PLAYER_ADD_DRIVER([garminnmea],[yes],[],[],[])

PLAYER_ADD_DRIVER([imageseq],[yes],[],[],[],[OPENCV],[opencv])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $OPENCV_LIBS"

PLAYER_ADD_DRIVER([isense],[yes],[isense/isense.h],
                  [],["-lisense"])

PLAYER_ADD_DRIVER([iwspy],[yes],[],[],[])

PLAYER_ADD_DRIVER([khepera],[yes],[],[],[])

PLAYER_ADD_DRIVER([laserbar],[yes],[],[],[])

PLAYER_ADD_DRIVER([laserbarcode],[yes],[],[],[])

PLAYER_ADD_DRIVER([lasercspace],[yes],[],[],[])

PLAYER_ADD_DRIVER([laserposeinterpolator],[yes],[],[],[])

PLAYER_ADD_DRIVER([laserrescan],[yes],[],[],[])

PLAYER_ADD_DRIVER([lasersafe],[yes],[],[],[])

PLAYER_ADD_DRIVER([laservisualbarcode],[yes],[],[],[])

PLAYER_ADD_DRIVER([laservisualbw],[yes],[],[],[])

PLAYER_ADD_DRIVER([linuxjoystick],[yes],[linux/joystick.h],[],[])

PLAYER_ADD_DRIVER([linuxwifi],[yes],[linux/wireless.h],
                  [],[],[],[],[[#include <netinet/in.h>]])

PLAYER_ADD_DRIVER([lifomcom],[no],[],[],[])

PLAYER_ADD_DRIVER([laserposeinterpolator],[yes],[],[],[])

PLAYER_ADD_DRIVER([logfile],[yes],[],[],[])

PLAYER_ADD_DRIVER([mapcspace],[yes],[],[],[])

PLAYER_ADD_DRIVER([mapfile],[yes],[],
                  [],[],[GDK_PIXBUF],[gdk-pixbuf-2.0])

PLAYER_ADD_DRIVER([mapscale],[yes],[],
                  [],[],[GDK_PIXBUF],[gdk-pixbuf-2.0])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $GDK_PIXBUF_LIBS"

PLAYER_ADD_DRIVER([microstrain],[yes],[],[],[])

PLAYER_ADD_DRIVER([mixer],[no],[sys/soundcard.h],[],[])

PLAYER_ADD_DRIVER([nomad],[no],[],[],[])

PLAYER_ADD_DRIVER([obot],[yes],[],[],[])

PLAYER_ADD_DRIVER([p2os],[yes],[],[],[])

PLAYER_ADD_DRIVER([erratic],[yes],[],[],[])

PLAYER_ADD_DRIVER([wbr914],[yes],[linux/serial.h],[],[])

PLAYER_ADD_DRIVER([passthrough],[no],[],[],
                  ["../client_libs/c/playercclient.o"])

PLAYER_ADD_DRIVER([ptu46],[yes],[],[],[])

PLAYER_ADD_DRIVER([reb],[no],[],[],[])

PLAYER_ADD_DRIVER([relay],[yes],[],[],[])

PLAYER_ADD_DRIVER([rflex],[yes],[],[],[])

dnl Where's CANLIB?
AC_ARG_WITH(canlib, [  --with-canlib=dir       Location of CANLIB],
CANLIB_DIR=$with_canlib,CANLIB_DIR=NONE)
if test "x$CANLIB_DIR" = "xNONE" -o "x$CANLIB_DIR" = "xno"; then
  SEGWAYRMP_HEADER=canlib.h
  SEGWAYRMP_EXTRA_CPPFLAGS=
  SEGWAYRMP_EXTRA_LDFLAGS=-lcanlib
else
  SEGWAYRMP_HEADER=$CANLIB_DIR/include/canlib.h
  SEGWAYRMP_EXTRA_CPPFLAGS="-I$CANLIB_DIR/include"
  SEGWAYRMP_EXTRA_LDFLAGS="-L$CANLIB_DIR/lib -lcanlib"
fi

PLAYER_ADD_DRIVER([segwayrmp],[no],
  [$SEGWAYRMP_HEADER], [$SEGWAYRMP_EXTRA_CPPFLAGS],
  [$SEGWAYRMP_EXTRA_LDFLAGS])

dnl Service Discovery with libhowl (mdns/zeroconf/rendezvous implementation)
PLAYER_ADD_DRIVER([service_adv_mdns],[no],
                  [],[],[],[HOWL],[howl >= 0.9.6])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $HOWL_LIBS"

PLAYER_ADD_DRIVER([shapetracker],[yes],[],[],[],
                  [OPENCV],[opencv])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $OPENCV_LIBS"

PLAYER_ADD_DRIVER([sicklms200],[yes],[],[],[])
if  test "x$enable_sicklms200" = "xyes"; then
  AC_CHECK_HEADERS(linux/serial.h, [], [], [])
fi

PLAYER_ADD_DRIVER([sicknav200],[yes],[],[],[])

PLAYER_ADD_DRIVER([sickpls],[yes],[],[],[])
if  test "x$enable_sickpls" = "xyes"; then
        AC_CHECK_HEADERS(linux/serial.h, [], [], [])
fi

PLAYER_ADD_DRIVER([sicks3000],[yes],[],[],[])
if  test "x$enable_sicks3000" = "xyes"; then
        AC_CHECK_HEADERS(linux/serial.h, [], [], [])
fi

AC_ARG_ENABLE(highspeedsick, [  --disable-highspeedsick   Don't build support for 500Kbps comms with SICK],,enable_highspeedsick=yes)
if test "x$enable_highspeedsick" = "xno"; then
  AC_DEFINE(DISABLE_HIGHSPEEDSICK,1,[disable 500Kbps comms with SICK])
fi

PLAYER_ADD_DRIVER([simpleshape],[yes],
                  [],[],[],[OPENCV],[opencv])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $OPENCV_LIBS"

PLAYER_ADD_DRIVER([sphere],[yes],[linux/videodev.h],[],[])

PLAYER_ADD_DRIVER([sphinx2],[yes],["sphinx2/CM_macros.h"],
                  [],["-lsphinx2 -lsphinx2fe -lsphinx2ad"])

PLAYER_ADD_DRIVER([sonyevid30],[yes],[],[],[])

PLAYER_ADD_DRIVER([upcbarcode],[yes],[],[],[],
                  [OPENCV],[opencv])
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $OPENCV_LIBS"

PLAYER_ADD_DRIVER([urglaser],[yes],[],[],[])
if  test "x$enable_urglaser" = "xyes"; then
  AC_CHECK_HEADERS(linux/serial.h, [], [], [])
fi

PLAYER_ADD_DRIVER([vfh],[yes],)

PLAYER_ADD_DRIVER([vmapfile],[yes],[],[],[])

PLAYER_ADD_DRIVER([waveaudio],[no],[sys/soundcard.h],[],[])

PLAYER_ADD_DRIVER([roomba],[yes],[],[],[])

PLAYER_ADD_DRIVER([wavefront],[yes],[],[],[])

PLAYER_ADD_DRIVER([yarpimage],[yes],["yarp/os/all.h yarp/sig/all.h"],[],["-lYARP_sig -lYARP_OS"])

dnl RFID support
PLAYER_ADD_DRIVER([insideM300],[yes],[],[],[])
PLAYER_ADD_DRIVER([skyetekM1],[yes],[],[],[])

dnl WSN support
PLAYER_ADD_DRIVER([mica2],[yes],[],[],[])
PLAYER_ADD_DRIVER([rcore_xbridge],[yes],[libparticle.h],[],["-lparticle"])

dnl The wavefront driver can make use of MD5 hash functions, if present
AC_ARG_ENABLE(md5, [  --disable-md5      Don't use MD5 hashing functions],,
enable_md5=yes)
if test "x$enable_md5" = "xyes"; then
  AC_CHECK_HEADERS(openssl/md5.h)
  AC_CHECK_LIB(crypto,MD5_Init)
fi





AC_DEFUN([AC_CXX_NAMESPACES],
[AC_CACHE_CHECK(whether the compiler implements namespaces,
ac_cv_cxx_namespaces,
[AC_LANG_SAVE
 AC_LANG_CPLUSPLUS
 AC_TRY_COMPILE([namespace Outer { namespace Inner { int i = 0; }}],
                [using namespace Outer::Inner; return i;],
 ac_cv_cxx_namespaces=yes, ac_cv_cxx_namespaces=no)
 AC_LANG_RESTORE
])
if test "$ac_cv_cxx_namespaces" = yes; then
  AC_DEFINE(HAVE_NAMESPACES,,[define if the compiler implements namespaces])
fi
])

AC_DEFUN([AX_BOOST_SIGNALS],
[AC_REQUIRE([AC_CXX_NAMESPACES])dnl
dnl Now determine the appropriate file names
AC_ARG_WITH([boost-signals],AS_HELP_STRING([--with-boost-signals],
  [specify the boost signals library or suffix to use]),
  [with_boost_signals=$withval],[with_boost_signals="yes"])
if test "x$with_boost_signals" != "xno"; then
  AC_CACHE_CHECK(whether the Boost::Signal library is available,
      ax_cv_boost_signal,
      [AC_LANG_SAVE
        AC_LANG_CPLUSPLUS
        AC_COMPILE_IFELSE(AC_LANG_PROGRAM([[#include <boost/signal.hpp>]],
          [[boost::signal<void ()> sig; return 0;]]),
          ax_cv_boost_signal="yes", ax_cv_boost_signal="no")
        AC_LANG_RESTORE
      ])
  if test "x$with_boost_signals" != "xyes"; then
    ax_boost_signals_lib="boost_signals-$with_boost_signals $with_boost_signals"
  fi
  for ax_lib in $ax_boost_signals_lib boost_signals boost_signals-gcc-mt ; do
    AC_CHECK_LIB($ax_lib, main, [BOOST_SIGNALS_LIB=$ax_lib break])
  done
  dnl make sure we have a lib
  if test $BOOST_SIGNALS_LIB; then
    if test "x$ax_cv_boost_signal" = "xyes"; then
      AC_DEFINE(HAVE_BOOST_SIGNALS,,[define if the Boost::Signal library is available])
    fi
  else
    ax_cv_boost_signal="no"
  fi
fi])
AC_SUBST(BOOST_SIGNALS_LIB)
])dnl

AC_DEFUN([AX_BOOST_THREAD],
[AC_REQUIRE([AC_CXX_NAMESPACES])dnl
dnl Now determine the appropriate file names
AC_ARG_WITH([boost-thread],AS_HELP_STRING([--with-boost-thread],
  [specify the boost thread library or suffix to use]),
  [with_boost_thread=$withval],[with_boost_thread="yes"])
if test "x$with_boost_thread" != "xno"; then
  AC_DEFINE(_POSIX_PTHREAD_SEMANTICS, 1, [Define pthread semantics])
  AC_DEFINE(_REENTRANT, 1, [Define reentrant])
  AC_CACHE_CHECK(whether the Boost::Thread library is available,
      ax_cv_boost_thread,
      [AC_LANG_SAVE
        AC_LANG_CPLUSPLUS
        AC_COMPILE_IFELSE(AC_LANG_PROGRAM([[#include "confdefs.h"
                                            #include <boost/thread/thread.hpp>]],
                 [[boost::thread thread; return 0;]]),
                 ax_cv_boost_thread="yes", ax_cv_boost_thread="no")
        CXXFLAGS=$CXXFLAGS_SAVE
        AC_LANG_RESTORE
      ])
  if test "x$with_boost_thread" != "xyes"; then
    ax_boost_thread_lib="boost_thread-$with_boost_thread $with_boost_thread"
  fi
  for ax_lib in $ax_boost_thread_lib boost_thread boost_thread-mt boost_thread-gcc-mt ; do
    AC_CHECK_LIB($ax_lib, main, [BOOST_THREAD_LIB=$ax_lib break])
  done
  dnl make sure we have a lib
  if test $BOOST_THREAD_LIB; then
    if test "x$ax_cv_boost_thread" = "xyes"; then
        AC_DEFINE(HAVE_BOOST_THREAD,,[define if the Boost::Thread library is available])
    fi
  else
    ax_cv_boost_thread="no"
  fi
fi])
AC_SUBST(BOOST_THREAD_LIB)
])dnl


dnl Add results from driver tests to compiler and link lines
PLAYER_DRIVER_EXTRA_LIBS="$PLAYER_DRIVER_EXTRA_LIBS $OPENCV_LIBS"

AC_SUBST(PLAYER_DRIVER_EXTRA_LIBS)

])

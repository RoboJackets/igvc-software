Installation Instructions:

Basic Installation:

  1) Make libopenvidia.a library.

  type: 

    make 

  2) Make some example programs

  go into the directory "examples" and type "make", or:

    cd examples
    make

  this will make executables: 
    'cgExample' : an example of using a fragment shader on an image file.
    'Dc1394example' : and example of loading video from a firewire webcam
    'V4L1example' : and example of loading video from a usb/video4linux webcam
    'feature' : example of feature detection and matching  (press 'r' to reset)
    'FLfeature' : a "scene manager" for recognizing multiple scenes

  3) Make some documentation (optional)
  
  If you have doxygen installed, you can generate the class documentation:
  
    make documentation

=====================================================================

What you'll need:
    * nvidia drivers

    You likely have already the following, otherwise you'll need to install

    * glut
    * imlib-dev Imlib development libraries
    * libxml-dev
    * libdc1394-dev (and 1394 support (raw1394 or video1394))
    * jpeg-mmx

    You'll likely have to install the following before compiling Openvidia:

    * Cg Compiler (cgc) (from nvidia.com website)
     
    and (http://www.gnu.org/directory/libs/cpp/commoncpp.html)
    * libcommoncpp2-1.0-0c102 (available as apt package for easier install)
    * libcommoncpp2-dev

    * libfltk1.1   (fast light toolkit for FLfeature)
    * libfltk1.1-dev

	----------------------------------------------------------
	Try the install_dep.sh script to install dependencies!
	Feel free to modify this script if I left out any!
	~chris
	----------------------------------------------------------
	

====================================================================


Example Programs:

Included in the /examples directory are some example programs:

1) cgExample

usage: ./cgExample FP-basic.cg someimage.ppm

This will apply the FP-basic.cg program to the image 'someimage.ppm' showing
the result on screen.  


2) Dc1394example: this program shows how to process video from firewire cameras.

3) featureExample: this program shows how to hook up the feature tracking
object to a firewire camera inputs.

4) feature: this program shows how to use the recovered features to do
some initial RANSAC processing and recover correspondences and a projective
coordinate transformation between an initial frame of video, and subsequent 
frames.



===================================================================

Programming :

To Compile your own programs with libopenvidia, look at the 
file examples/Makefile, which has the necessary flags which need
to be included 

See openvidia.org or openvidia.sf.net for more examples and 


===================================================================


Additional Help:
  
   http://openvidia.sf.net has forums to post to and read.

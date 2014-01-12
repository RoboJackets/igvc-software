Latest revision : Dec. 18, 2013

Welcome to the RoboJackets IGVC software repo!
This document will give you a brief description of the repo's layout and some simple instructions for building the documentation and project. For more detailed information, please visit wiki.robojackets.org

Contents:
I   Folder Structure
II  Building Documentation
III Building Code


I. Folder Structure
The repo's root folder contains 7 folders and 1 file.
FOLDERS
 * 2013
    This folder is a copy of the software repo as it stood at the end of the 2013 season. Many changes occurred in the project setup since then. This folder is there as an easy access point to the old code base.
 * arduino
    This folder contains the code that runs on the arduino boards on the robot. These can be edited via gedit, but it is recommended that you use the Arduino IDE (www.arduino.cc) to build and upload the code to an arduino board.
 * ardupilot
    This folder contains the code that runs on the Ardupilot IMU device. This folder includes the Ardupilot library and make systems for building & deploying the code.
 * IGVC_Control
    This folder contains the project description files for Qt Creator.
 * src
    This folder contains the C++ source code that runs on the on-board laptop. The folder hierarchy under /src is very deep, but relatively self explanatory.
 * test_data
    This folder contains test data including images, videos, sensor logs, etc.
 * udev
    This folder contains udev rules used by our code to identify serial devices.
FILES
 * Doxyfile
    This is the configuration file for the doxygen documentation generation system (www.doxygen.org)


II. Building Documentation
To generate the documentation for our laptop-side source code, run the following command from the repo's root folder:
...$ doxygen Doxyfile
This will create a /docs folder in the repo's root directory which will contain HTML and LaTex versions of the documentation. To generate PDF documentation, run the following command from /docs/latex/
...$ make


III. Building Code
The easiest way to build the laptop-side code is to use Qt Creator (www.qt-project.org) to open /IGVC_Control/IGVC_Control.pro, configure the project with the standard settings, and click the build button (hammer icon).

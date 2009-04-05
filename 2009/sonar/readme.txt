================***SONARREADER_1_00 FOLDER***===============================================================================
Contains files and libs used for interfacing laptop to SonarRanger (arduino).

Important Files:
	SonarInterface.cc, SonarInterface.h
		Driver for interfacing to SonarRanger
	ArduinoInterface.cc, ArduinoInterface.h, ArduinoCmds.hpp, DataPacket.cc, DataPacket.hpp, DataPacketStructs.hpp
		External libs required for serial communication and std arduino interface 	(written by Jacob)
	sonartest_1_02.cc
		current 'demo' for the SonarInterface driver
	sonartest_1_01.cc, sonar_test.cc
		older test files -- not up to date, so ignore these
============================================================================================================================

================***SONARRANGER_3_03 FOLDER***===============================================================================
Contains source files to be exported to the Arduino. Handles sonar communication.

Important Files:
	SonarRanger_3_03.pde
		Main source file
	Arduino_SD.cpp, Arduino_SD.h
		External libs required for serial communication and command recognition (written by Jacob, altered by me)
	Sonar.c, Sonar.h
		External libs required for I2C interface (specific to SRF08 devices) and Sonar device handling
	twi.c, twi.h
		'Two Wire Interface' lib used for general I2C interface
============================================================================================================================

---------Last Updated: April 4, 2009-------

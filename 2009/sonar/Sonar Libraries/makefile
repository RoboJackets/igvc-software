all: sonardevice

sonardevice: sonartest_checkall.o SonarInterface.o ArduinoInterface.o DataPacket.o
	g++ -o sdev sonartest_checkall.o SonarInterface.o ArduinoInterface.o DataPacket.o

sonartest_1_01.o: sonartest_checkall.cc
	g++ -Wall -c -o sonartest_checkall.o sonartest_checkall.cc

SonarInterface.o: SonarInterface.cc
	g++ -Wall -c -o SonarInterface.o SonarInterface.cc

ArduinoInterface.o: ArduinoInterface.cc
	g++ -Wall -c -o ArduinoInterface.o ArduinoInterface.cc

DataPacket.o: DataPacket.cc
	g++ -Wall -c -o DataPacket.o DataPacket.cc


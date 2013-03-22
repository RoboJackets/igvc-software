#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string>
#include <iostream>

#include "serial/ASIOSerialPort.h"


using namespace std;
/*
int main()
{
    ASIOSerialPort xbee("/dev/ttyUSB0", 57600);
    //ASIOSerialPort xbee("/dev/ttyUSB0", 9600);
    string testString = "0123456789";
    while (1)
    {
        usleep(5000);
        xbee.write(testString);
    }

}

*/

int main(int argc, char *argv[]){
    string newLine;

    cout << "Usage: takes in one parameter, the name of the file to write to, must specifify extension" << endl;
    sleep(5);

    cout << "Beginning logging" << endl;

    ofstream logFile;
    logFile.open(argv[1]);



    ASIOSerialPort xbee("/dev/ttyUSB1", 57600);

    while(1)
    {
        try
        {
            newLine = xbee.readln();
        }
        catch(...)
        {
            continue;
        }
        if (newLine != "")
        {
            cout << newLine << endl;
            logFile << newLine << endl;
        }
    }
    return 0;

}


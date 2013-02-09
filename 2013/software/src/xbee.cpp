#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string>

#include "serial/ASIOSerialPort.h"


using namespace std;
int main(int argc, char *argv[]){
    string newLine;

    cout << "Usage: takes in one parameter, the name of the file to write to, must specifify extension" << endl;
    sleep(5);

    cout << "Beginning logging" << endl;

    ofstream logFile;
    logFile.open(argv[1]);



    ASIOSerialPort xbee("/dev/XBEE", 57600);

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

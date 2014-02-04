#include "gpsfilereader.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>
using namespace std;

// format of GPS coordinate: lat,long\n
void GPSFileReader::read(string file, queue<GPSData>& gpsList)
{
    string line = "";
    ifstream infile;
    infile.open(file.c_str());

    if(!infile.is_open())
    {
        stringstream msg;
        msg << "Could not open file (in GPSFileReader): " << file << endl;
        cout << msg;
        Logger::Log(LogLevel::Error, msg.str());
        throw GPSFileNotFoundException(msg.str());
    }

    int lineIndex = 1;
    while(!infile.eof())
    {
        getline(infile, line);
        if (line.length() > 0)
        {
            vector<string> tokens = split(line, ',');

            //file validation
            if (tokens.size() != 2)
            {
                stringstream msg;
                msg << "GPSFileReader: Entry at line " << lineIndex << " is malformed. (Size != 2)" << endl;
                Logger::Log(LogLevel::Error, msg.str());
                throw GPSFileFormatException(msg.str());
            }

            GPSData newData;
            newData.Lat(atof(tokens.at(0).c_str()));
            newData.Long(atof(tokens.at(1).c_str()));

            //coordinate validation
            if (abs(newData.Lat()) > 90)
            {
                stringstream msg;
                msg << "GPSFileReader: Latitude at line " << lineIndex << " is not a valid latitude (" << newData.Lat() << ")" << endl;
                Logger::Log(LogLevel::Error, msg.str());
                throw GPSFileFormatException(msg.str());
            }
            if (abs(newData.Long()) > 180)
            {
                stringstream msg;
                msg << "GPSFileReader: Longitude at line " << lineIndex << " is not a valid longitude (" << newData.Long() << ")" << endl;
                Logger::Log(LogLevel::Error, msg.str());
                throw GPSFileFormatException(msg.str());
            }
            gpsList.push(newData);
            lineIndex++;
        }
    }
    infile.close();
}

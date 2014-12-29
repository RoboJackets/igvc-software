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
    if(file.empty())
    {
        throw GPSFileNotFoundException("Cannot load empty file path.");
    }

    string line = "";
    ifstream infile;
    infile.open(file.c_str());

    if(!infile.is_open())
    {
        string msg = "[GPSFileReader] Could not open file: " + file;
        Logger::Log(LogLevel::Error, msg);
        throw GPSFileNotFoundException(msg);
    }

    int lineIndex = 1;
    while(!infile.eof())
    {
        getline(infile, line);
        if (line.length() > 0)
        {
            vector<string> tokens = split(line, ',');

            //file validation
            if (tokens.size() != 5)
            {
                string msg = "[GPSFileReader] Entry at line " + to_string(lineIndex) + " is malformed. (Size != 2)";
                Logger::Log(LogLevel::Error, msg);
                throw GPSFileFormatException(msg);
            }

            GPSData newData;
            newData.Lat(atof(tokens.at(0).c_str()));
            newData.Long(atof(tokens.at(1).c_str()));
            newData.NumSats(atof(tokens.at(2).c_str()));
            newData.Quality((GPS_QUALITY)atoi(tokens.at(3).c_str()));
            newData.HDOP(atof(tokens.at(4).c_str()));

            //coordinate validation
            if (abs(newData.Lat()) > 90)
            {
                string msg = "[GPSFileReader] Latitude at line " + to_string(lineIndex) + " is not a valid latitude (" + to_string(newData.Lat()) + ").";
                Logger::Log(LogLevel::Error, msg);
                throw GPSFileFormatException(msg);
            }
            if (abs(newData.Long()) > 180)
            {
                string msg = "[GPSFileReader] Longitude at line " + to_string(lineIndex) + " is not a valid longitude (" + to_string(newData.Long()) + ").";
                Logger::Log(LogLevel::Error, msg);
                throw GPSFileFormatException(msg);
            }
            gpsList.push(newData);
            lineIndex++;
        }
    }
    infile.close();
}

#ifndef GPSFILEREADER_H
#define GPSFILEREADER_H
#include <string>
#include <queue>
#include "GPS.hpp"
using namespace std;

class GPSFileReader
{
public:
    GPSFileReader();
    GPSFileReader(string file);
    GPSData getNext();
private:
    queue<GPSData> gpsList;
};

#endif // GPSFILEREADER_H

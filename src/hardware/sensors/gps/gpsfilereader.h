#ifndef GPSFILEREADER_H
#define GPSFILEREADER_H
#include <string>
#include <queue>
#include "GPS.hpp"
using namespace std;

class GPSFileReader
{
public:
    static void read(string file, queue<GPSData>& gpsList);

private:
};

#endif // GPSFILEREADER_H

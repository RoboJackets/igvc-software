#ifndef GPSFILEREADER_H
#define GPSFILEREADER_H
#include <string>
#include <queue>
#include <common/datastructures/GPSData.hpp>
using namespace std;

class GPSFileReader
{
public:
    static void read(string file, queue<GPSData>& gpsList);

private:
};

struct GPSFileFormatException {
    std::string message;
    GPSFileFormatException(std::string msg) : message(msg) { }
};

struct GPSFileNotFoundException {
    std::string message;
    GPSFileNotFoundException(std::string msg) : message(msg) { }
};

#endif // GPSFILEREADER_H

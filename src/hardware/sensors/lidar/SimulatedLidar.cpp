#include "hardware/sensors/lidar/SimulatedLidar.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <QErrorMessage>
#include <sstream>

SimulatedLidar::SimulatedLidar()
    : _data()
{
    _running = true;
    _thread = boost::thread(boost::bind(&SimulatedLidar::thread_run, this));
    _delay = 500000;

    for(int i = 0; i < 1024; i++)
    {
        LidarPoint &p = _data.points[i];
        p.valid = true;
        p.angle = i * ( ( M_PI * 2.0 ) / 1024);
        //p.distance = i/1024.0; // Makes a cool spiral
        //p.distance = 1; // Makes a circle
        //p.distance = 0.5*(3*sin(p.angle)-sqrt(9*sin(p.angle)*sin(p.angle) + 10)); // Circle centered above origin
        p.distance = 0.5*(2*sin(p.angle)-sqrt(4*sin(p.angle)*sin(p.angle) - 3));
        if(p.angle < M_PI / 3.0 || p.angle > 2.0*M_PI/3.0)
            p.valid = false;
    }
}

void SimulatedLidar::thread_run()
{
    while(_running)
    {
        try {
            sleep(1);
            boost::this_thread::interruption_point();
            onNewData(_data);
        } catch (const boost::thread_interrupted&) {
            _running = false;
            break;
        }
    }
}

SimulatedLidar::~SimulatedLidar()
{
    _thread.interrupt();
}

LidarState SimulatedLidar::GetState()
{
    return _data;
}

bool SimulatedLidar::IsWorking()
{
    return true;
}

void SimulatedLidar::loadFile(const char *path)
{
    _running = false;
    _thread.join();
    using namespace std;
    LidarState state;
    string line;
    ifstream file;
    file.open(path);
    if(file.is_open())
    {
        // Read and ignore first line of file
        // The first line is just column labels
        getline(file,line);
        // Loop through and populate the points
        // list from the remaining lines
        for(int i = 0; i < 1024 && file.good(); i++)
        {
            getline(file,line);
            vector<string> tokens = StringSplit(line, ",", true);

            LidarPoint &pt = state.points[atoi(tokens[0].c_str())];
            pt.valid = atoi(tokens[1].c_str());
            pt.angle = atof(tokens[2].c_str());
            pt.raw = atoi(tokens[3].c_str());
            pt.distance = atof(tokens[4].c_str());
            pt.intensity = atoi(tokens[5].c_str());
        }
        file.close();
    } else {
        stringstream msg;
        msg << "Could not open file: ";
        msg << path;
        QErrorMessage errmsg;
        errmsg.showMessage(msg.str().c_str());
        errmsg.exec();
    }
    _data = state;
    _thread = boost::thread(boost::bind(&SimulatedLidar::thread_run, this));
    _running = true;
}

void SimulatedLidar::setDelay(int usec)
{
    _delay = usec;
}

/**
 * Tokenizes a string based on the given delimeter
 * source - The string to tokenize
 * delimeter - The character to mark splits
 * keepEmpty - Flag that controls whether or not this method will add empty strings to the results
 *             For example, the source "hello,,world" with keepEmpty = true, will return "hello","","world"
 * This method was copied from an answer to a StackOverflow answer found here:
 *      http://stackoverflow.com/questions/10051679/c-tokenize-string
 */
std::vector<std::string> SimulatedLidar::StringSplit(const std::string &source, const char *delimiter, bool keepEmpty)
{
    std::vector<std::string> results;

    size_t prev = 0;
    size_t next = 0;

    while ((next = source.find_first_of(delimiter, prev)) != std::string::npos)
    {
        if (keepEmpty || (next - prev != 0))
        {
            results.push_back(source.substr(prev, next - prev));
        }
        prev = next + 1;
    }

    if (prev < source.size())
    {
        results.push_back(source.substr(prev));
    }

    return results;
}


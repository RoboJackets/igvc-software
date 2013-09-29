#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <QDateTime>

namespace LogLevel
{
    enum Level {
        Debug,
        Info,
        Warning,
        Error
    };
    static const char* LevelNames[] = {"Debug","Info","Warning","Error"};
    static std::string str(Level level)
    {
        return LevelNames[level];
    }
}

class Logger
{
public:

    template<typename T>
    static inline void Log(LogLevel::Level level, T msg)
    {
        if(!_instance)
            _instance = new Logger();
        std::stringstream output;
        output << dateTimeStr() << " [" << LogLevel::str(level) << "] \t";
        output << msg;
        if(level == LogLevel::Error)
        {
            std::cerr << output.str() << std::endl;
            std::cerr.flush();
        }
        else if(level != LogLevel::Debug)
        {
            std::cout << output.str() << std::endl;
            std::cout.flush();
        }
        _instance->_fileOutStream << output.str() << std::endl;
        _instance->_fileOutStream.flush();
    }

private:

    static std::string dateTimeStr();

    static Logger *_instance;

    std::ofstream _fileOutStream;

    Logger();

    ~Logger();
};

#endif // LOGGER_H

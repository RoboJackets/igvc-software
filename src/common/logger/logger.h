#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <QDateTime>
#include <QStatusBar>

namespace LogLevel
{
    enum Level {
        Debug,
        Info,
        Warning,
        Error
    };
    /* Names are padded so that all strings have the same length.
     * This makes printed formatting nicer.
     */
    static const char* LevelNames[] = {"Debug  ", "Info   ", "Warning", "Error  "};
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
            if(_instance->_statusBar)
            {
                _instance->_statusBar->clearMessage();
                _instance->_statusBar->setStyleSheet("QStatusBar{padding-left:8px;background:rgba(0,0,0,0);color:red;font-weight:bold;}");
                _instance->_statusBar->showMessage(output.str().c_str());
            }
        }
        else if(level != LogLevel::Debug)
        {
            std::cout << output.str() << std::endl;
            std::cout.flush();
//            if(_instance->_statusBar)
//            {
//               _instance->_statusBar->clearMessage();
//               _instance-> _statusBar->setStyleSheet("QStatusBar{padding-left:8px;background:rgba(0,0,0,0);color:black;font-weight:bold;}");
//               _instance-> _statusBar->showMessage(output.str().c_str());
//            }
        }
        _instance->_fileOutStream << output.str() << std::endl;
        _instance->_fileOutStream.flush();
    }

    static void setSatusBar(QStatusBar *statusBar)
    {
        if(!_instance)
            _instance = new Logger();
        _instance->_statusBar = statusBar;
    }

    static void Clear();

private:

    static std::string dateTimeStr();

    static Logger *_instance;

    std::ofstream _fileOutStream;

    QStatusBar *_statusBar;

    Logger();

    ~Logger();
};

#endif // LOGGER_H

#include "logger.h"
#include <QDir>

Logger *Logger::_instance;

Logger::Logger()
{
    QDir dir;
    dir.mkdir(QDir::currentPath() + "/Logs/");
    std::string logpath = QDir::currentPath().toStdString() +
            "/Logs/Log_" +
            QDateTime::currentDateTime().toString("yyy-M-d_hh-mm-ss-zzz").toStdString() +
            ".txt";
    std::cout << logpath << std::endl;
    _fileOutStream.open(logpath.c_str());
    _statusBar = 0;
}

Logger::~Logger()
{
    _fileOutStream.close();
    _instance = 0;
}

std::string Logger::dateTimeStr()
{
    return QDateTime::currentDateTime().toString("yyyy-M-d_hh-mm-ss-zzz").toStdString();
}

void Logger::Clear()
{
    if(!_instance)
        _instance = new Logger();
    QDir dir(QDir::currentPath() + "/Logs/");
    QStringList files = dir.entryList();
    files.sort();
    for(int i = 0; i < files.size()-1; i++)
        if(!QFile(files[i]).remove())
            Log(LogLevel::Error, "Could not clear all log files.");
    Log(LogLevel::Info, "Log files cleared.");
}

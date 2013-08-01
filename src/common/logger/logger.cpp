#include "logger.h"
#include <QDir>

Logger *Logger::_instance;

Logger::Logger()
{
    QDir dir;
    dir.mkdir(QDir::currentPath() + "/Logs/");
    std::stringstream stream;
    stream << QDir::currentPath().toStdString();
    stream << "/Logs/Log_";
    stream << QDateTime::currentDateTime().toString("yyyy-M-d_hh-mm-ss-zzz").toStdString();
    stream << ".txt";
    std::cout << stream.str() << std::endl;
    _fileOutStream.open(stream.str().c_str());
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

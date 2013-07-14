#include "configmanager.h"

using namespace std;

bool ConfigManager::load(std::string path)
{
    QFile file(QString(path.c_str()));

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        cout << "[ConfigManager] Could not open config file: " << path << endl;
        return false;
    }

    if(!xmlFile.setContent(&file))
    {
        cout << "[ConfigManager] Could not parse config file: " << path << endl;
        return false;
    }

    file.close();
    return true;
}

bool ConfigManager::save(string path)
{
    QFile file(QString(path.c_str()));

    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        cout << "[ConfigManager] Could not open file." << endl;
        return false;
    }

    QTextStream fileStream(&file);

    xmlFile.save(fileStream, 4);

    return true;
}

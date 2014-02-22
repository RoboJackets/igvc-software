#include "configmanager.h"
#include "common/logger/logger.h"

using namespace std;

string ConfigManager::defaultPath = "";

ConfigManager::ConfigManager()
{
    QString path = QDir::currentPath();
    path += "/config.xml";
    defaultPath = path.toStdString();
}

bool ConfigManager::load(std::string path)
{
    if(!path.empty())
        defaultPath = path;

    QFile file(QString(defaultPath.c_str()));

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        stringstream msg;
        msg << "[ConfigManager] Could not open config file: " << defaultPath;
        Logger::Log(LogLevel::Warning, msg.str());
        //cout << "[ConfigManager] Could not open config file: " << defaultPath << endl;
        return false;
    }

    if(!xmlFile.setContent(&file))
    {
        stringstream msg;
        msg << "[ConfigManager] Could not parse config file: " << defaultPath;
        Logger::Log(LogLevel::Warning, msg.str());
        //cout << "[ConfigManager] Could not parse config file: " << defaultPath << endl;
        return false;
    }

    file.close();
    StructureChanged();
    return true;
}

bool ConfigManager::save(string path)
{
    if(path.empty())
        path = defaultPath;

    QFile file(QString(path.c_str()));

    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        stringstream msg;
        msg << "[ConfigManager] Could not open file for saving.";
        Logger::Log(LogLevel::Warning, msg.str());
        //cout << "[ConfigManager] Could not open file for saving." << endl;
        return false;
    }

    QTextStream fileStream(&file);

    xmlFile.save(fileStream, 4);

    return true;
}

int ConfigManager::numberOfCategories()
{
    return xmlFile.documentElement().childNodes().count();
}

int ConfigManager::numberOfValues(string category)
{
    if(!xmlFile.documentElement().firstChildElement(category.c_str()).isNull())
        return xmlFile.documentElement().firstChildElement(category.c_str()).childNodes().count();
    else
        return -1;
}

int ConfigManager::numberOfValues(int categoryIndex)
{
    if(categoryIndex < 0 || categoryIndex >= xmlFile.documentElement().childNodes().count())
    {
        return -1;
    }
    else
    {
        return xmlFile.documentElement().childNodes().at(categoryIndex).childNodes().count();
    }
}

string ConfigManager::categoryLabel(int categoryInd)
{
    if(categoryInd < xmlFile.documentElement().childNodes().count())
    {
        return xmlFile.documentElement().childNodes().at(categoryInd).nodeName().toStdString();
    }
    return "";
}

string ConfigManager::valueLabel(int categoryInd, int valueInd)
{
    QDomElement root = xmlFile.documentElement();
    if(categoryInd < root.childNodes().count())
    {
        QDomNode categoryNode = root.childNodes().at(categoryInd);
        if(valueInd < categoryNode.childNodes().count())
        {
            return categoryNode.childNodes().at(valueInd).nodeName().toStdString();
        }
    }
    return "";
}

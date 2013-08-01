#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <QtCore>
#include <QtXml>
#include <string>
#include <sstream>
#include <iostream>

class ConfigManager
{
public:
    static ConfigManager& Instance()
    {
        static ConfigManager instance;

        return instance;
    }

    bool load(std::string path = defaultPath);

    template<typename T>
    T getValue(std::string category, std::string name, T defaultVal)
    {
        using namespace std;
        QDomElement root = xmlFile.documentElement();

        QDomElement categoryNode = root.firstChildElement(category.c_str());

        if(categoryNode.isNull())
        {
            cout << "[ConfigManager] No such category." << endl;
            root.appendChild(xmlFile.createElement(category.c_str()));
            root.firstChildElement(category.c_str()).appendChild(xmlFile.createElement(name.c_str()));
            stringstream stream;
            stream << defaultVal;
            root.firstChildElement(category.c_str()).firstChildElement(name.c_str()).appendChild(xmlFile.createTextNode(stream.str().c_str()));
            return defaultVal;
        }

        QDomElement variableNode = categoryNode.firstChildElement(name.c_str());

        if(variableNode.isNull())
        {
            cout << "[ConfigManager] No such variable." << endl;
            categoryNode.appendChild(xmlFile.createElement(name.c_str()));
            stringstream stream;
            stream << defaultVal;
            categoryNode.firstChildElement(name.c_str()).appendChild(xmlFile.createTextNode(stream.str().c_str()));
            return defaultVal;
        }

        string value = variableNode.firstChild().nodeValue().toStdString();

        stringstream stream;

        stream << value;

        T result;

        stream >> result;

        return result;
    }

    template<typename T>
    T getValue(int categoryInd, int nameInd, T defaultVal)
    {
        using namespace std;
        QDomElement root = xmlFile.documentElement();

        if(categoryInd >= root.childNodes().count())
            return defaultVal;

        QDomNode categoryNode = root.childNodes().at(categoryInd);

        if(nameInd >= categoryNode.childNodes().count())
            return defaultVal;

        string value = categoryNode.childNodes().at(nameInd).firstChild().nodeValue().toStdString();

        stringstream stream;

        stream << value;

        T result;

        stream >> result;

        return result;
    }

    template<typename T>
    void setValue(std::string category, std::string name, T value)
    {
        using namespace std;
        stringstream stream;

        stream << value;

        QDomElement root = xmlFile.documentElement();

        QDomElement categoryNode = root.firstChildElement(category.c_str());

        if(categoryNode.isNull())
        {
            cout << "[ConfigManager] No such category." << endl;
            root.appendChild(xmlFile.createElement(category.c_str()));
            root.firstChildElement(category.c_str()).appendChild(xmlFile.createElement(name.c_str()));
            root.firstChildElement(category.c_str()).firstChildElement(name.c_str()).appendChild(xmlFile.createTextNode(stream.str().c_str()));
            return;
        }

        QDomElement variableNode = categoryNode.firstChildElement(name.c_str());

        if(variableNode.isNull())
        {
            cout << "[ConfigManager] No such variable." << endl;
            root.firstChildElement(category.c_str()).appendChild(xmlFile.createElement(name.c_str()));
            root.firstChildElement(category.c_str()).firstChildElement(name.c_str()).appendChild(xmlFile.createTextNode(stream.str().c_str()));
            return;
        }

        variableNode.firstChild().setNodeValue(stream.str().c_str());
    }

    template<typename T>
    void setValue(int categoryInd, int nameInd, T value)
    {
        using namespace std;
        stringstream stream;

        stream << value;

        QDomElement root = xmlFile.documentElement();

        if(categoryInd < root.childNodes().count())
        {
            QDomNode categoryNode = root.childNodes().at(categoryInd);
            if(nameInd < categoryNode.childNodes().count())
            {
                categoryNode.childNodes().at(nameInd).firstChild().setNodeValue(stream.str().c_str());
            }
        }
    }

    bool save(std::string path = defaultPath);

    int numberOfCategories();

    int numberOfValues(std::string category);

    int numberOfValues(int categoryIndex);

    std::string categoryLabel(int categoryInd);

    std::string valueLabel(int categoryInd, int valueInd);

protected:

    QDomDocument xmlFile;

    static std::string defaultPath;

private:
    ConfigManager();

    ConfigManager(ConfigManager const&);

    void operator=(ConfigManager const);
};

#endif // CONFIGMANAGER_H

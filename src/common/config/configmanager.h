#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <QtCore>
#include <QtXml>
#include <string>
#include <sstream>
#include <iostream>
#include <QObject>

/*!
 * \brief The ConfigManager class
 *
 * This class allows for reading from and writing to an XML configuration file.
 * Configuration files are useful for storing values that may change frequently, such as controller coefficients and physical device dimensions.
 * Variables are represented as a two-level tree of categorized variables. This allows for duplicating variable names without ambiguity.
 *
 * \author Matthew Barulic
 * \headerfile configmanager.h <common/configmanager.h>
 */
class ConfigManager : public QObject
{
    Q_OBJECT
public:
    static ConfigManager& Instance()
    {
        static ConfigManager instance;

        return instance;
    }

    /*!
     * \brief load Loads the configuration settings from the given path, or the default path if none is given.
     * \param path The path to load the configuration settings from.
     * \return True if successful, false otherwise.
     */
    bool load(std::string path = defaultPath);

    /*!
     * \brief getValue Returns the config value with the given category and name.
     * \param category The category to search under.
     * \param name The name of the value to fetch.
     * \param defaultVal The default value to use if no value exists with the given category and name.
     * \return The value of the config variable or defaultVal if none was found.
     */
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
            StructureChanged();
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
            StructureChanged();
            return defaultVal;
        }

        string value = variableNode.firstChild().nodeValue().toStdString();

        stringstream stream;

        stream << value;

        T result;

        stream >> result;

        return result;
    }

    /*!
     * \brief getValue Returns the config value with the given category and name.
     * \param categoryInd The index of the category to search under.
     * \param nameInd The index of the value to fetch.
     * \param defaultVal The default value to use if no value exists with the given category and name.
     * \return The value of the config variable or defaultVal if none was found.
     */
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

    /*!
     * \brief setValue Stores the given value in the specified config variable.
     * \param category The name of the category to store under.
     * \param name The name of the variable to store under.
     * \param value The value to store.
     *
     * If either the category or variable do not already exist, they will be created and the given value saved.
     */
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
            StructureChanged();
            return;
        }

        QDomElement variableNode = categoryNode.firstChildElement(name.c_str());

        if(variableNode.isNull())
        {
            cout << "[ConfigManager] No such variable." << endl;
            root.firstChildElement(category.c_str()).appendChild(xmlFile.createElement(name.c_str()));
            root.firstChildElement(category.c_str()).firstChildElement(name.c_str()).appendChild(xmlFile.createTextNode(stream.str().c_str()));
            StructureChanged();
            return;
        }

        variableNode.firstChild().setNodeValue(stream.str().c_str());
    }

    /*!
     * \brief setValue Stores the given value in the specified config variable.
     * \param categoryInd The index of the category to store under.
     * \param nameInd The index of the variable to store under.
     * \param value The value to store.
     *
     * If either the category or variable indeces are out of bounds the value will not be saved.
     */
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

    /*!
     * \brief save Saves the configuration data to the given file path.
     * \param path The path to save the config data.
     * \return True if successful, false otherwise
     */
    bool save(std::string path = defaultPath);

    int numberOfCategories();

    /*!
     * \brief numberOfValues Returns the number of variables under the given category.
     */
    int numberOfValues(std::string category);

    /*!
     * \brief numberOfValues Returns the number of variables under the category at the given index.
     */
    int numberOfValues(int categoryIndex);

    /*!
     * \brief categoryLabel Returns the name of the category at the given index.
     */
    std::string categoryLabel(int categoryInd);

    /*!
     * \brief valueLabel Returns the name of the variable at the given category / value index pair.
     */
    std::string valueLabel(int categoryInd, int valueInd);

signals:
    /*!
     * \brief StructureChanged This signal is emitted every time the structure of the config tree is changed.
     */
    void StructureChanged();

protected:

    QDomDocument xmlFile;

    static std::string defaultPath;

private:
    ConfigManager();

    ConfigManager(ConfigManager const&);

    void operator=(ConfigManager const);
};

#endif // CONFIGMANAGER_H

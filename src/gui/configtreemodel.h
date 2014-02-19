#ifndef CONFIGGUIADAPTER_H
#define CONFIGGUIADAPTER_H

#include <QStandardItemModel>

/*!
 * \brief Handles translating configuration data for the config editor tree in MainWindow.
 * \author Matthew Barulic
 */
class ConfigTreeModel : public QObject
{
    Q_OBJECT
public:
    ConfigTreeModel(QObject* parent = new QObject());

    /*!
     * \brief Fills the model with data from ConfigManager
     */
    void populateModel();

    QStandardItemModel* model();

public slots:
    /*!
     * \brief Saves changed data to the ConfigManager
     * \param item The item in the model whose value changed
     * \note This will attempt to save te config data to disk.
     */
    void dataWasChanged(QStandardItem* item);

private:
    QStandardItemModel _model;
};

#endif // CONFIGGUIADAPTER_H

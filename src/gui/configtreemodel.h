#ifndef CONFIGGUIADAPTER_H
#define CONFIGGUIADAPTER_H

#include <QStandardItemModel>
#include <common/events/Event.hpp>

class ConfigTreeModel : public QObject
{
    Q_OBJECT
public:
    ConfigTreeModel(QObject* parent = new QObject());

    void populateModel();

    QStandardItemModel* model();

public slots:
    void dataWasChanged(QStandardItem* item);

private:
    QStandardItemModel _model;

    void OnConfigStructureChanged(void*);
    LISTENER(ConfigTreeModel, OnConfigStructureChanged, void*);
};

#endif // CONFIGGUIADAPTER_H

#ifndef CONFIGGUIADAPTER_H
#define CONFIGGUIADAPTER_H

#include <QStandardItemModel>

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
};

#endif // CONFIGGUIADAPTER_H

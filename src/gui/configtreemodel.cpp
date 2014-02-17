#include "configtreemodel.h"
#include "common/config/configmanager.h"

#include <QMessageBox>

class ConfigTreeItem
{
public:
    ConfigTreeItem(int cID, int vID, bool label)
    {
        categoryID = cID;
        valueID = vID;
        isLabel = label;
    }

    int categoryID;
    int valueID;
    bool isLabel;

    bool operator== (const ConfigTreeItem item) const
    {
        return categoryID == item.categoryID &&
               valueID == item.valueID &&
               isLabel == item.isLabel;
    }
};

ConfigTreeModel::ConfigTreeModel(QObject *)
{
    connect(&_model, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(dataWasChanged(QStandardItem*)));
    connect(&ConfigManager::Instance(), &ConfigManager::StructureChanged, [=](){
        populateModel();
    });
}

void ConfigTreeModel::dataWasChanged(QStandardItem *item)
{
    int cInd = item->parent()->row();
    int vInd = item->row();
    ConfigManager::Instance().setValue(cInd, vInd, _model.data(item->index()).toString().toStdString());
    ConfigManager::Instance().save();
}

void ConfigTreeModel::populateModel()
{
    _model.clear();
    _model.setColumnCount(2);
    _model.setHeaderData(0, Qt::Horizontal, "Name");
    _model.setHeaderData(1, Qt::Horizontal, "Value");
    ConfigManager &config = ConfigManager::Instance();
    QStandardItem *rootNode = _model.invisibleRootItem();
    for(int cInd = 0; cInd < config.numberOfCategories(); cInd++)
    {
        QStandardItem *categoryNode = new QStandardItem(config.categoryLabel(cInd).c_str());
        // Make category rows not selectable, only expandable
        categoryNode->setFlags(categoryNode->flags() & ~Qt::ItemIsSelectable & ~Qt::ItemIsEditable);
        QStandardItem *categoryValuePlaceholder = new QStandardItem();
        categoryValuePlaceholder->setFlags(Qt::ItemIsEnabled);
        QList<QStandardItem*> catList;
        catList.append(categoryNode);
        catList.append(categoryValuePlaceholder);
        rootNode->appendRow(catList);
        for(int vInd = 0; vInd < config.numberOfValues(cInd); vInd++)
        {
            QStandardItem *valueLabelNode = new QStandardItem(config.valueLabel(cInd, vInd).c_str());
            valueLabelNode->setFlags(valueLabelNode->flags() & ~Qt::ItemIsSelectable & ~Qt::ItemIsEditable);
            QStandardItem *valueValueNode = new QStandardItem(config.getValue(cInd, vInd, std::string()).c_str());
            valueValueNode->setFlags(valueValueNode->flags() | Qt::ItemIsEditable);
            QList<QStandardItem*> row;
            row.append(valueLabelNode);
            row.append(valueValueNode);
            categoryNode->appendRow(row);
        }
    }
}

QStandardItemModel* ConfigTreeModel::model()
{
    return &_model;
}

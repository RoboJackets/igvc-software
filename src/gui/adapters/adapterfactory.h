#ifndef ADAPTERFACTORY_H
#define ADAPTERFACTORY_H
#include <QWidget>
#include <memory>
#include <common/module.hpp>

class AdapterFactory
{
public:
    static QWidget* getAdapterForModule(std::shared_ptr<Module> module, QWidget *parent = 0);

};

#endif // ADAPTERFACTORY_H

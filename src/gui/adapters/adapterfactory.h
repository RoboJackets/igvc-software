#ifndef ADAPTERFACTORY_H
#define ADAPTERFACTORY_H
#include <QWidget>
#include <common/module.hpp>

class AdapterFactory
{
public:
    AdapterFactory();

    QWidget* getAdapterForModule(const Module &module);

};

#endif // ADAPTERFACTORY_H

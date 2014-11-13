#ifndef COORDINATOR_HPP
#define COORDINATOR_HPP

#include <QObject>
#include <vector>
#include <memory>
#include <common/module.hpp>
#include <common/datastructures/MotorCommand.hpp>

typedef std::vector< std::shared_ptr<Module> > module_list_t;

class Coordinator : public QObject {
    Q_OBJECT
public:
    virtual const module_list_t& getModules() = 0;

    virtual std::shared_ptr<Module> getModuleWithName(std::string name) = 0;

    virtual ~Coordinator() { }

signals:
    void newMotorCommand(MotorCommand cmd);
    void finished();

public slots:
    virtual void changeLidar(std::shared_ptr<Module>) { }
    virtual void changeGPS(std::shared_ptr<Module>) { }
    virtual void run() { }
    virtual void pause() { }
    virtual void stop() { }
};

#endif // COORDINATOR_HPP

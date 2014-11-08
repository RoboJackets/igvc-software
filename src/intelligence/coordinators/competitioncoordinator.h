#ifndef COMPETITIONCOORDINATOR_H
#define COMPETITIONCOORDINATOR_H

#include "coordinator.hpp"

class CompetitionCoordinator : public Coordinator
{
    Q_OBJECT
public:
    CompetitionCoordinator();

    ~CompetitionCoordinator();

    virtual const module_list_t &getModules();

    virtual std::shared_ptr<Module> getModuleWithName(std::string name);

public slots:
    virtual void changeLidar(std::shared_ptr<Module>);
    virtual void changeGPS(std::shared_ptr<Module>);

signals:
    void newMotorCommand(MotorCommand cmd);

private:
    module_list_t modules;
    module_list_t::iterator find(std::string name);
};

#endif // COMPETITIONCOORDINATOR_H

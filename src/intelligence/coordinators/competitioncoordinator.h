#ifndef COMPETITIONCOORDINATOR_H
#define COMPETITIONCOORDINATOR_H

#include "coordinator.hpp"

class CompetitionCoordinator : public Coordinator
{
public:
    CompetitionCoordinator();

    ~CompetitionCoordinator();

    virtual const module_list_t &getModules();

    virtual std::shared_ptr<Module> getModuleWithName(std::string name);

private:
    module_list_t modules;
};

#endif // COMPETITIONCOORDINATOR_H

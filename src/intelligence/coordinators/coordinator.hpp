#ifndef COORDINATOR_HPP
#define COORDINATOR_HPP

#include <vector>
#include <memory>
#include <common/module.hpp>

typedef std::vector< std::shared_ptr<Module> > module_list_t;

class Coordinator {
public:
    virtual const module_list_t& getModules() = 0;

    virtual std::shared_ptr<Module> getModuleWithName(std::string name) = 0;

    virtual ~Coordinator() { }
};

#endif // COORDINATOR_HPP

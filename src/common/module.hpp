#ifndef MODULE_HPP
#define MODULE_HPP

#include <QObject>

/*!
 * \brief The Module class is the base class for all of the nodes in our event graph.
 * \author Matthew Barulic
 * \headerfile module.hpp <common/module.hpp>
 */
class Module : public QObject {

Q_OBJECT

public:

    /*!
     * \brief Gives the status of this module.
     * \return True if this module is working correctly. False otherwise.
     */
    virtual bool isWorking() = 0;

};

#endif // MODULE_HPP

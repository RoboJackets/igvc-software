// =============================================================================
// Event Class for .NET Style Events
// by Clint Caywood
// February 18, 2010
// =============================================================================

#ifndef EVENT_HPP
#define EVENT_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include "Delegate.hpp"


/*!
 * \brief The Event class
 *
 * This class handles registration and removal of listeners as well as the firing of the event itself.
 * To use this class, create an instance of this type and expose it as a public member so other classes can register listeners.
 * \note This class was originally downloaded from an article by Clint Caywood. Some minor modifications have since been made.
 * \headerfile Event.hpp <common/events/Event.hpp>
 */
template <typename T>
class Event
{
    typedef typename std::vector< Delegate<T>* >::iterator iter;
    public:
        void operator+=(Delegate<T>* delegate)
        {
            // An object can only subscribe once
            if (find(_delegates.begin(), _delegates.end(), delegate) == _delegates.end())
            {
                _delegates.push_back(delegate);
            }
        }
        void operator-=(Delegate<T>* delegate)
        {
            iter i = _delegates.begin();
            if(_delegates.size() == 0)
                return;
            while (i != _delegates.end())
            {
                if (*i == delegate)
                {
                    i = _delegates.erase(i);
                }
                else
                {
                    ++i;
                }
            }
        }
        void operator()(T param)
        {
            for (iter i = _delegates.begin(); i != _delegates.end(); ++i)
            {
                if((*i) != NULL)
                (*i)->operator()(param);
            }
        }

        typename std::vector<Delegate<T>* >::size_type numDelegates()
        {
            return _delegates.size();
        }

        void Clear()
        {
            _delegates.clear();
        }

    private:
        std::vector< Delegate<T>* > _delegates;
};

#endif

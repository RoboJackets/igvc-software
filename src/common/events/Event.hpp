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

    private:
        std::vector< Delegate<T>* > _delegates;
};

#endif

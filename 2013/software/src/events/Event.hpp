// =============================================================================
// Event Class for .NET Style Events
// by Clint Caywood
// February 18, 2010
// =============================================================================

#ifndef EVENT_HPP
#define EVENT_HPP

#include <vector>
#include <algorithm>
#include "Delegate.hpp"

template <typename T>
class Event
{
    public:
        inline void operator+=(Delegate<T>* delegate)
        {
            // An object can only subscribe once
            if (find(_delegates.begin(), _delegates.end(), delegate) == _delegates.end())
            {
                _delegates.push_back(delegate);
            }
        }
        inline void operator-=(Delegate<T>* delegate)
        {
            typedef typename std::vector< Delegate<T>* >::iterator iter;
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
        inline void operator()(T param)
        {
            typedef typename std::vector< Delegate<T>* >::iterator iter;
            for (iter i = _delegates.begin(); i != _delegates.end(); ++i)
            {
                (*i)->operator()(param);
            }
        }

    private:
        std::vector< Delegate<T>* > _delegates;
};

#endif

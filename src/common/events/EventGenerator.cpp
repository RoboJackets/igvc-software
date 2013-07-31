#include "EventGenerator.h"

template <class ListenerType>
EventGenerator<ListenerType>::EventGenerator()
{
}

template <class ListenerType>
EventGenerator<ListenerType>::~EventGenerator()
{
    //dtor
}

template <class ListenerType>
void EventGenerator<ListenerType>::addListener(ListenerType *listener)
{
    listeners.push_back(listener);
}

template <class ListenerType>
void EventGenerator<ListenerType>::removeListener(ListenerType *listener)
{
    listeners.remove(listener);
}

template <class ListenerType>
void EventGenerator<ListenerType>::fireEvent( void (ListenerType::*eventFunction) (void*), void* eventData )
{
    for(typename std::list<ListenerType*>::iterator iter = listeners.begin(); iter != listeners.end(); iter++)
    {
        ((*iter)->*eventFunction)(eventData);
    }
}

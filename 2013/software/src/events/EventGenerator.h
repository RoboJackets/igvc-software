#ifndef EVENTGENERATOR_H
#define EVENTGENERATOR_H

#include <list>

/** Base class for managing event listeners and triggering events. */
template <class ListenerType>
class EventGenerator
{
    public:
        /** Default constructor */
        EventGenerator();
        /** Default destructor */
        virtual ~EventGenerator();

        void addListener(ListenerType* listener);
        void removeListener(ListenerType* listener);
        void fireEvent( void (ListenerType::*eventFunction) (void*), void* eventData );
    private:
        std::list<ListenerType*> listeners;
};

#endif // EVENTGENERATOR_H

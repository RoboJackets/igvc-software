//
// F7BPPChangedEvent.h
//
//

#ifndef F7BPPCHANGEDEVENT_H
#define F7BPPCHANGEDEVENT_H

#include "qevent.h"
#include <CustomEvents.h>


class F7BPPChangedEvent:public QCustomEvent {
public:
    F7BPPChangedEvent():QCustomEvent(F7BPPCHANGED) {};
};


#endif // F7BPPCHANGEDEVENT_H

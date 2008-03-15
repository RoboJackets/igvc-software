//
// UpdateF7Event.h
//
//

#ifndef UPDATEF7EVENT_H
#define UPDATEF7EVENT_H

#include "qevent.h"


class UpdateF7Event:public QCustomEvent {
public:
    UpdateF7Event():QCustomEvent(60522) {};
    
};


#endif // UPDATEF7EVENT_H

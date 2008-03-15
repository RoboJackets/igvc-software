//
// CamWindowCloseEvent.h
//
//

#ifndef CAMWINCLOSEEVENT_H
#define CAMWINCLOSEEVENT_H

#include "qevent.h"
#include "CustomEvents.h"


class CamWindowCloseEvent:public QCustomEvent {
public:
    CamWindowCloseEvent():QCustomEvent(CAMWINDOWCLOSE) {};
    
};


#endif // CAMWINCLOSEEVENT_H

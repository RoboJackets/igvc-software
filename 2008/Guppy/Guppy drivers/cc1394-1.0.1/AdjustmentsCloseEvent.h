//
// AdjustmentsCloseEvent.h
//
//

#ifndef ADJUSTMENTSCLOSEEVENT_H
#define ADJUSTMENTSCLOSEEVENT_H

#include "qevent.h"
#include "CustomEvents.h"


class AdjustmentsCloseEvent:public QCustomEvent {
public:
    AdjustmentsCloseEvent():QCustomEvent(ADJUSTMENTSCLOSE) {};
    
};


#endif // ADJUSTMENTSCLOSEEVENT_H

//
// F7UpdateNotifyEvent.h
//
//

#ifndef F7UPDATENOTIFYEVENT_H
#define F7UPDATENOTIFYEVENT_H

#include "qevent.h"
#include <F7NotifyData.h>
#include <CustomEvents.h>


class F7UpdateNotifyEvent:public QCustomEvent {
public:
    F7UpdateNotifyEvent():QCustomEvent(F7UPDATENOTIFY) {};
    F7UpdateNotifyEvent(F7NotifyData data):
            QCustomEvent(F7UPDATENOTIFY), newGeometry(data) {};
    
    F7NotifyData geometry() {return newGeometry;};
    
private:
    F7NotifyData newGeometry;
    
};


#endif // F7UPDATENOTIFYEVENT_H

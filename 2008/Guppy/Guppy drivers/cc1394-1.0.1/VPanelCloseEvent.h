//
// VPanelCloseEvent.h
//
//

#ifndef VPANELCLOSEEVENT_H
#define VPANELCLOSEEVENT_H

#include "qevent.h"


class VPanelCloseEvent:public QCustomEvent {
public:
    VPanelCloseEvent():QCustomEvent(66521) {};
    
};


#endif // VPANELCLOSEEVENT_H

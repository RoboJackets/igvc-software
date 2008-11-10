
#include "urg_laser.h"


int main() {
    UrgLaser L;
    L.initialize();	
    L.makeScan(0,768,1);		
}




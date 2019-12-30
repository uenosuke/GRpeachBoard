#ifndef AUTO_h
#define AUTO_h

#include "PathTracking.h"
#include "define.h"

class Auto{
    public:
    Auto();
    void pathTrackingMode(int mode, int state, int nextPhase);
    void commandMode(int nextPhase, boolean next = true);
    void getSwState(int num);
    void getRefVel();

    private:
};

#endif
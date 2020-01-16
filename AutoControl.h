#ifndef AUTOCONTROL_h
#define AUTOCONTROL_h

#include "PathTracking.h"
#include "define.h"
#include "SDclass.h"

//#define PATHNUM     ( 50 )
//#define POINTNUM    ( 100 )

class AutoControl{
    public:
    AutoControl();
    int init(mySDclass*, int);
    void gPosiInit();
    coords pathTrackingMode(int mode, int state, int nextPhase);
    void commandMode(int nextPhase, boolean next = true);
    coords getRefVel();

    int phase = 0;
    int swState = 0;

    // mainプログラムとPathTrackingの媒介的な
    double Px(int);
    double Py(int);
    double onx();
    double ony();
    double angle();
    double dist();
    double refKakudo();
    void initSettings();
    void setConvPara(double conv_length, double conv_tnum);
    void setMaxPathnum(int);
    int getPathNum();


    private:
};

#endif
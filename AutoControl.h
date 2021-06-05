#ifndef AUTOCONTROL_h
#define AUTOCONTROL_h

#include "PathTracking.h"
#include "define.h"
#include "SDclass.h"

class AutoControl{
    public:
    AutoControl();
    int init(mySDclass*, int);
    void gPosiInit();
    coords pathTrackingMode(int mode, int state, int nextPhase);
    void calibrationGposi(double tempX, double tempY, double tempZ);
    coords commandMode_vel(double tempX, double tempY, double tempZ);
    void commandMode(int nextPhase, boolean next = true);
    coords getRefVel(unsigned int swState = 0);

    int phase = 0;
    
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
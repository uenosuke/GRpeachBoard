#ifndef PIDCLASS_h
#define PIDCLASS_h

#include "Arduino.h"

class PID
{
public:
    PID(float xKp, float xKi, float xKd, float xint_time);
    float getCmd(float ref, float act, float maxcmd);
    float PIDinit(float ref, float act);
    void setPara(float xKp, float xKi, float xKd);

private:
    float preError;
    float intError;
    float Kp;
    float Ki;
    float Kd;
    float int_time;
    
    boolean init_done;
    
};

#endif
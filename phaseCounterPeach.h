#ifndef PHASECOUNTER_h
#define PHASECOUNTER_h

//#include"gr_common/rx63n/iodefine_gcc63n.h"
#include "RZ_A1H.h"

#define ENC_INIT_VAL 0x7FFF

class phaseCounter
{
public:
    phaseCounter(int);
    int getCount();
    void setCount(int ch, int num);
    void init();

private:
    bool ch_available;
    
    unsigned short int pre_rawcount;
    int encount;
    
    int g_ch;
};

#endif

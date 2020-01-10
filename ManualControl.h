#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"

#define JOY_DEADBAND    ( 5 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( 2.5 )

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    ManualControl();
    
    coords getVel(unsigned int, unsigned int, unsigned int);

private:
    
};

#endif
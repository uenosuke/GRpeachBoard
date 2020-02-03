#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    ManualControl();
    
    coords getRefVel(unsigned int, unsigned int, unsigned int);

private:
    bool anglePIDEnable;
    
};

#endif
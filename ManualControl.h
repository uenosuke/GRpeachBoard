// ゲームコントローラのジョイスティックデータから，
// 指令速度を計算するクラス
// 作成日：2019年12月30日
// 作成者：上野祐樹

#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"

#define JOY_DEADBAND    ( 5 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( 2.5 )

class ManualControl{
public:
    /*********** 変数宣言 ***********/
    

    double refVx, refVy, refVz;
    double refKakudo;
    double tmpPx, tmpPy;

    /*********** 関数宣言 ***********/
    ManualControl();
    
    coords getVel(unsigned int, unsigned int, unsigned int);

private:
    int path_num;
    int mode;
    int max_pathnum;

    double conv_length;
    double conv_tnum;

    bool mode_changed;
    bool init_done;

    double tan, per, rot;
};

#endif
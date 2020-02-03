#ifndef PLATFORM_h
#define PLATFORM_h

#include "define.h"

#if DRIVE_UNIT == PLATFORM_DUALWHEEL
    // 双輪キャスター関連
    #define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
    #define RADIUS_R    ( 0.04 )    // wheel radius
    #define RADIUS_L    ( 0.04 )    // wheel radius
    #define W           ( 0.265 )    // tread
    #define GEARRATIO   ( 5.5 )
    #define TT_RES4     ( 4096 )    // turntableの分解能
    #define _2RES_PI    ( 2 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能)  
    #define _2RES_PI_T  ( 2 * 500 / 3.141592 ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(500はエンコーダ分解能)
#elif DRIVE_UNIT == PLATFORM_MECHANUM
    // メカナム関連
    #define MECANUM_HANKEI		( 0.05 )    //車輪半径
    #define MECANUM_HANKEI_D	( 0.15561 ) //車輪幅
    #define MECANUM_HANKEI_L	( 0.26023 ) //前後幅
    #define _2RES_PI  ( 2.0 * MECANUM_RES / PI ) //  駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数
#elif DRIVE_UNIT == PLATFORM_OMNI3WHEEL
    #define _2RES_PI    ( 2 * 3 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数  
    #define WHEEL_R		( 0.019 )
    #define DIST2WHEEL  ( 0.120 )
    #define GEARRATIO   ( 51.45 )
    #define COS_PI_6    ( 0.86602540378 )
    #define SIN_PI_6    ( 0.5 )
#endif

// RoboClaw関連
#define ADR_MD1             ( 128 )
#define ADR_MD2             ( 129 )

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * 3.141592 / 800 ) // res = 200
#define RADIUS_X    ( 0.024 )
#define RADIUS_Y    ( 0.024 )

class Platform{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    Platform();
    void platformInit(coords);
    void setPosi(coords);
    coords getPosi(int, int, double);
    void VelocityControl(coords);

private:
    coords Posi;
    int preEncX, preEncY;
    double pre_angle_rad;
    bool init_done;
    #if DRIVE_UNIT == PLATFORM_DUALWHEEL
        int stateamt203;
    #endif
};

#endif
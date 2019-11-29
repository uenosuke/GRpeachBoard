#include "PIDclass.h"

// コンストラクタPIDパラメータを引数を用いて初期化する
PID::PID(float xKp, float xKi, float xKd, float xint_time)
{
    Kp = xKp;
    Ki = xKi;
    Kd = xKd;
    int_time = xint_time;

    preError = 0.0; // 1個前のエラーの値
    intError = 0.0; // 積分値の初期化

    init_done = false;
}

float PID::PIDinit(float ref, float act)
{
    preError = ref - act;
    intError = 0.0; // 積分値の初期化

    init_done = true;
}

// PID制御の実体部
float PID::getCmd(float ref, float act, float maxcmd)
{
    float cmd, Error, dError;
    cmd = 0.0;

    if(init_done) {
        Error = ref - act;
        cmd += Error * Kp;

        dError = (Error - preError);// / int_time; int_timeが0.01のときdErrorの値が大きくなりすぎてしまうのでコメントアウト
        cmd += dError * Kd;

        intError += (Error + preError) / 2 * int_time;
        cmd += intError * Ki;

        preError = Error;

        if(cmd > maxcmd) cmd = maxcmd;
        if(cmd < -maxcmd) cmd = -maxcmd;
    }
    return cmd;
}

void PID::setPara(float xKp, float xKi, float xKd)
{
    Kp = xKp;
    Ki = xKi;
    Kd = xKd;
}
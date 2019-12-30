#include "Auto.h"

PathTracking motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードでとりあえず初期化

Auto::Auto(){
}

void pathTrackingMode(int mode, int state, int nextPhase){
    if(motion.getMode() != mode) motion.setMode(mode);
    syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
    
    if(syusoku == 1){ // 収束して次の曲線へ
        if( pathNum <= state ){
            motion.Px[3*pathNum+3] = gPosix;
            motion.Py[3*pathNum+3] = gPosiy;
            motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

            if( pathNum == state ) phase = nextPhase;
        }
    }else if(syusoku == 0){ // まだ収束していない，軌道追従中
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
    }else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
    }
}

void commandMode(int nextPhase, boolean next/*=true*/){
    if( next ){
        motion.Px[3*pathNum+3] = gPosix;
        motion.Py[3*pathNum+3] = gPosiy;
        motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
    }else{
        motion.Px[3*pathNum] = gPosix;
        motion.Py[3*pathNum] = gPosiy;
    }
    
    phase = nextPhase;
}

// リミットスイッチ押すまでX方向に動き続けるといった処理がある場合に使用
// 必要に応じて名前を変えたりしてください
void getSwState(int num){
    swState = num;
}

// このメソッドの中身はユーザーが書き換える必要あり
void getRefVel(){
    // example >>>>>
    if( phase == 0 ){
        pathTrackingMode(FOLLOW_TANGENT, 7, 1);
    }else if( phase == 1 ){
        pathTrackingMode(FOLLOW_COMMAND, 8, 2);
    }else if( phase == 2 ){
        // 下のように速度指令値を与える場合はrefVel_optionを使用
        refVx = 0.5;
        refVy = 0.0;
        refVz = 0.0;
        if(swState = 0b0001){
            commandMode(3);
            // gPosix = 3.0; // 位置のキャリブレーション
            // gPosiy = 3.0;
            // gPosiz = 3.0;
        }
    }else{
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
    }
    // <<<<<
}
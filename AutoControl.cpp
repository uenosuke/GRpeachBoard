#include "AutoControl.h"

PathTracking motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードでとりあえず初期化

AutoControl::AutoControl(){
}

// SDのデータを読み込んで，PathTrackingの変数に格納
int AutoControl::init(mySDclass* mySD, int FieldColor){
    mySD->path_read(FieldColor, motion.Px  , motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
}

coords AutoControl::pathTrackingMode(coords gPosi, int mode, int state, int nextPhase){
    coords refV;
    int pathNum = getPathNum();

    if(motion.getMode() != mode) motion.setMode(mode);
    int syusoku = motion.calcRefvel(gPosi.x, gPosi.y, gPosi.z); // 収束していれば　1　が返ってくる
    
    if(syusoku == 1){ // 収束して次の曲線へ
        if( pathNum <= state ){
            motion.Px[3*pathNum+3] = gPosi.x;
            motion.Py[3*pathNum+3] = gPosi.y;
            motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

            if( pathNum == state ) phase = nextPhase;
        }
    }else if(syusoku == 0){ // まだ収束していない，軌道追従中
        refV.x = motion.refVx;
        refV.y = motion.refVy;
        refV.z = motion.refVz;
    }else{ // それ以外は問題ありなので止める
        refV.x = 0.0;
        refV.y = 0.0;
        refV.z = 0.0;
    }
    return refV;
}

void AutoControl::commandMode(coords gPosi, int nextPhase, boolean next/*=true*/){
    int pathNum = getPathNum();
    if( next ){
        motion.Px[3*pathNum+3] = gPosi.x;
        motion.Py[3*pathNum+3] = gPosi.y;
        motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
    }else{
        motion.Px[3*pathNum] = gPosi.x;
        motion.Py[3*pathNum] = gPosi.y;
    }
    
    phase = nextPhase;
}

// リミットスイッチ押すまでX方向に動き続けるといった処理がある場合に使用
// 必要に応じて名前を変えたりしてください
void AutoControl::getSwState(int num){
    swState = num;
}

double AutoControl::Px(int num){
    motion.Px[num];
}

double AutoControl::Py(int num){
    motion.Py[num];
}

int AutoControl::getPathNum(){
    return motion.getPathNum();
}

double AutoControl::onx(){
    return motion.onx;
}

double AutoControl::ony(){
    return motion.ony;
}

double AutoControl::angle(){
    return motion.angle;
}

double AutoControl::dist(){
    return motion.dist;
}

double AutoControl::refKakudo(){
    return motion.refKakudo;
}

void AutoControl::initSettings(){
    motion.initSettings();
}
void AutoControl::setConvPara(double conv_length, double conv_tnum){
    motion.setConvPara(conv_length, conv_tnum);
}
void AutoControl::setMaxPathnum(int pathNum){
    motion.setMaxPathnum(pathNum);
}

// このメソッドの中身はユーザーが書き換える必要あり
coords AutoControl::getRefVel(coords gPosi){
    coords refV;

    // example >>>>>
    if( phase == 0 ){
        refV = pathTrackingMode(gPosi, FOLLOW_TANGENT, 7, 1);
    }else if( phase == 1 ){
        refV = pathTrackingMode(gPosi, FOLLOW_COMMAND, 8, 2);
    }else if( phase == 2 ){
        // 下のように速度指令値を与える場合はrefVel_optionを使用
        refV.x = 0.5;
        refV.y = 0.0;
        refV.z = 0.0;
        if(swState = 0b0001){
            commandMode(gPosi, 3);
            // gPosi.x = 3.0; // 位置のキャリブレーション
            // gPosi.y = 3.0;
            // gPosi.z = 3.0;
        }
    }else{
        refV.x = 0.0;
        refV.y = 0.0;
        refV.z = 0.0;
    }

    return refV;
    // <<<<<
}
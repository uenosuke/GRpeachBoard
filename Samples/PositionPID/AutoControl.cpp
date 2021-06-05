#include "AutoControl.h"

extern coords gPosi;

PathTracking motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードでとりあえず初期化

AutoControl::AutoControl(){
}

// SDのデータを読み込んで，PathTrackingの変数に格納
int AutoControl::init(mySDclass* mySD, int FieldColor){
    return mySD->path_read(FieldColor, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
}

void AutoControl::gPosiInit(){
    gPosi.x = motion.Px[0];
    gPosi.y = motion.Py[0];
    gPosi.z = motion.refangle[0];
}

coords AutoControl::pathTrackingMode(int mode, int state, int nextPhase){ // 軌道追従モード
    coords refV;
    int pathNum = getPathNum();

    if(motion.getMode() != mode) motion.setMode(mode);
    int syusoku = motion.calcRefvel(); // 収束していれば　1　が返ってくる
    
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

void AutoControl::calibrationGposi(double tempX, double tempY, double tempZ){
    gPosi.x = tempX;
    gPosi.y = tempY;
    gPosi.z = tempZ;
}

coords AutoControl::commandMode_vel(double tempX, double tempY, double tempZ){
    coords refV;
    refV.x = tempX;
    refV.y = tempY;
    refV.z = tempZ;
    return refV;
}

void AutoControl::commandMode(int nextPhase, boolean next/*=true*/){ // 指定した速度で動かすとき
    int pathNum = getPathNum();

    if( next ){ // この動きを一つの曲線とみなす場合
        motion.Px[3*pathNum+3] = gPosi.x;
        motion.Py[3*pathNum+3] = gPosi.y;
        motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
    }else{
        motion.Px[3*pathNum] = gPosi.x;
        motion.Py[3*pathNum] = gPosi.y;
    }
    
    phase = nextPhase;
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
coords AutoControl::getRefVel(unsigned int swState){
    coords refV = {0.0, 0.0, 0.0};
    static unsigned int pre_swState = swState;

    // example of position PID >>>>>
    if( phase == 0 ){
        // ボタンが押されるまで待機
        if((swState != pre_swState) && (swState & MASK_BUTTON_A == MASK_BUTTON_A)){ // Aボタンが押されたら最初の目的地へ
            if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID); // 強制的に位置PIDモードにする
            motion.setConvPara(0.02, 0.997); // 次の位置へ．第1引数は収束半径，第2引数は収束したと判定するベジエ曲線のt値(位置制御では使わない)
            phase = 1;
        }
    }
    else if( phase == 1 ){
        int syusoku = motion.calcRefvel(); // 初期位置Px[0],Py[0]から，最終位置Px[3],Py[3],refangle[0]で位置PID制御
        refV.x = motion.refVx; // 計算された値を代入
        refV.y = motion.refVy;
        refV.z = motion.refVz;
        if((syusoku == 1) && ((swState != pre_swState) && (swState & MASK_BUTTON_A == MASK_BUTTON_A))){ // Aボタンが押されたら次の目的地へ
            motion.incrPathnum(0.02, 0.997); // 次の位置へ．第1引数は収束半径，第2引数は収束したと判定するベジエ曲線のt値(位置制御では使わない)
            phase = 2;
        }
    }
    else if(phase == 2){
        int syusoku = motion.calcRefvel(); // 初期位置Px[3],Py[3]から，最終位置Px[6],Py[6],refangle[1]で位置PID制御
        refV.x = motion.refVx; // 計算された値を代入
        refV.y = motion.refVy;
        refV.z = motion.refVz;
        if(syusoku == 1){ // こちらは収束したら自動で次の目的地へ
            motion.incrPathnum(0.02, 0.997); // 次の位置へ．第1引数は収束半径，第2引数は収束したと判定するベジエ曲線のt値(位置制御では使わない)
            phase = 3;
        }
    }
    else if(phase == 3){
        int syusoku = motion.calcRefvel(); // 初期位置Px[6],Py[6]から，最終位置Px[9],Py[9],refangle[2]で位置PID制御
        refV.x = motion.refVx; // 計算された値を代入
        refV.y = motion.refVy;
        refV.z = motion.refVz;
        if((syusoku == 1) && ((swState != pre_swState) && (swState & MASK_BUTTON_B == MASK_BUTTON_B))){ // Bボタンが押されたら次の目的地へ
            motion.incrPathnum(0.02, 0.997); // 次の位置へ．第1引数は収束半径，第2引数は収束したと判定するベジエ曲線のt値(位置制御では使わない)
            motion.setPathNum(0); // 最初の目的地へ
            phase = 1; // 以降，繰り返し
        }
    }
    // <<<<<

    else{
        refV = commandMode_vel(0.0, 0.0, 0.0); // 該当しない場合はとりあえず速度ゼロ
    }

    pre_swState = swState;

    return refV;
}
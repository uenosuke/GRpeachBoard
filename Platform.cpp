// 4輪オムニやメカナム，双輪キャスタなどのプラットフォームごとに
// 各モータの指令速度の計算からRoboClawの制御，自己位置推定を行なうクラス
// Platform.h の DRIVE_UNIT の定義でプラットフォームを選択する
// 作成日：2019年12月30日
// 作成者：上野祐樹

#include "Arduino.h"
#include "Platform.h"
#include "RoboClaw.h"
#include "AMT203VPeach.h"

RoboClaw MD(&SERIAL_ROBOCLAW,1);

#if DRIVE_UNIT == PLATFORM_DUALWHEEL
    AMT203V amt203(&SPI, PIN_CSB);
#endif

Platform::Platform(){
    init_done = false;
}

// 自己位置推定の初期化
void Platform::platformInit(coords initPosi){
#if DRIVE_UNIT == PLATFORM_DUALWHEEL
    SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
    SPI.setClockDivider(SPI_CLOCK_DIV16); //SPI通信のクロックを1MHzに設定 beginの後に置かないと，処理が止まる
    stateamt203 = amt203.init();
#endif

    MD.begin(115200);

    Posi = initPosi;

    preEncX = 0;
    preEncY = 0;
    pre_angle_rad = 0.0;//Posi.z;
    init_done = true;
}

// 自己位置推定値(Posi)を外部からセット
void Platform::setPosi(coords tempPosi){
    Posi = tempPosi;
}

// エンコーダのカウント値と，ジャイロセンサから取得した角度をもとに自己位置を計算する
coords Platform::getPosi(int encX, int encY, double angle_rad){
    if(init_done){
        // ローカル座標系での変化量を計算(zは角度)
        coords diff;

        // エンコーダのカウント値から角度の変化量を計算する
        double angX, angY;
        angX = (double)( encX - preEncX ) * _2PI_RES4;
        angY = (double)( encY - preEncY ) * _2PI_RES4;
        
        double angle_diff;
        angle_diff = angle_rad - pre_angle_rad; // 角度の変化量を計算
        diff.z = angle_diff;
        diff.x = RADIUS_X * angX; //RADIUS_X はX軸エンコーダの車輪半径
        diff.y = RADIUS_Y * angY; //RADIUS_Y はY軸エンコーダの車輪半径

        // グローバル座標系での変化量に変換し，これまでのデータに加算することで自己位置推定完了
        Posi.z += diff.z;
        Posi.x += diff.x * cos( Posi.z ) - diff.y * sin( Posi.z );
        Posi.y += diff.x * sin( Posi.z ) + diff.y * cos( Posi.z );
        
        // 1サンプル前のデータとして今回取得したデータを格納
        preEncX = encX;
        preEncY = encY;
        pre_angle_rad = angle_rad;
    }
    return Posi;
}

void Platform::VelocityControl(coords refV){
    if(init_done){
        #if DRIVE_UNIT == PLATFORM_OMNI3WHEEL
            double refOmegaA, refOmegaB, refOmegaC;

            refOmegaA = (-refV.y - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
            refOmegaB = ( refV.x*COS_PI_6 + refV.y*SIN_PI_6 - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
            refOmegaC = (-refV.x*COS_PI_6 + refV.y*SIN_PI_6 - refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;

            // RoboClawの指令値に変換
            double mdCmdA, mdCmdB, mdCmdC;
            mdCmdA = refOmegaA * _2RES_PI;
            mdCmdB = refOmegaB * _2RES_PI;
            mdCmdC = refOmegaC * _2RES_PI;

            // モータにcmdを送り，回す
            MD.SpeedM1(ADR_MD1, (int)mdCmdA);// 右前
            MD.SpeedM2(ADR_MD1, (int)mdCmdB);// 左前
            MD.SpeedM2(ADR_MD2, (int)mdCmdC);// 右後
        #elif DRIVE_UNIT == PLATFORM_DUALWHEEL
            // ターンテーブルの角度取得
            double thetaDuEnc, thetaDu;
            static double  preThetaDuEnc = thetaDuEnc;
            thetaDuEnc = amt203.getEncount(); 
            if( thetaDuEnc == -1 ){
            thetaDuEnc = preThetaDuEnc; // -1はエラーなので，前の値を格納しておく
            }
            preThetaDuEnc = thetaDuEnc;
            thetaDu = (double)thetaDuEnc*2*PI / TT_RES4;	// 角度に変換
            
            // 車輪やターンテーブルの指令速度を計算
            double cosDu, sinDu, refOmegaR, refOmegaL, refOmegaT;
            cosDu = cos(thetaDu);
            sinDu = sin(thetaDu);
            refOmegaR = ( ( cosDu - sinDu ) * refV.x + ( sinDu + cosDu ) * refV.y ) / RADIUS_R;// right
            refOmegaL = ( ( cosDu + sinDu ) * refV.x + ( sinDu - cosDu ) * refV.y ) / RADIUS_L;// left
            refOmegaT = ( - ( 2 * sinDu / W ) * refV.x + ( 2 * cosDu / W ) * refV.y - refV.z ) * GEARRATIO;// turntable

            // RoboClawの指令値に変換
            double mdCmdR, mdCmdL, mdCmdT;
            mdCmdR = refOmegaR * _2RES_PI;
            mdCmdL = refOmegaL * _2RES_PI;
            mdCmdT = refOmegaT * _2RES_PI_T;

            // モータにcmdを送り，回す
            MD.SpeedM1(ADR_MD1, -(int)mdCmdR);// 右車輪
            MD.SpeedM2(ADR_MD1,  (int)mdCmdL);// 左車輪
            MD.SpeedM1(ADR_MD2,  (int)mdCmdT);// ターンテーブル
        #elif DRIVE_UNIT == PLATFORM_MECHANUM
            refOmegaA = ( refV.x - refV.y - refV.z * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;// 左前
            refOmegaB = ( refV.x + refV.y - refV.z * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;// 左後
            refOmegaC = ( refV.x - refV.y + refV.z * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;// 右後
            refOmegaD = ( refV.x + refV.y + refV.z * ( MECANUM_HANKEI_D + MECANUM_HANKEI_L ) ) / MECANUM_HANKEI;// 右前

            // RoboClawの指令値に変換
            double mdCmdA, mdCmdB, mdCmdC, mdCmdD;
            mdCmdA = refOmegaA * _2RES_PI;
            mdCmdB = refOmegaB * _2RES_PI;
            mdCmdC = refOmegaC * _2RES_PI;
            mdCmdD = refOmegaD * _2RES_PI;

            MD.SpeedM1(ADR_MD1, -(int)mdCmdA);// 右前
            MD.SpeedM2(ADR_MD1,  (int)mdCmdB);// 左前
            MD.SpeedM1(ADR_MD2,  (int)mdCmdC);// 右後
            MD.SpeedM2(ADR_MD2,  (int)mdCmdD);// 左後
        #endif
    }
}
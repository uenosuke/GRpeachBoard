#include "phaseCounterPeach.h"

phaseCounter::phaseCounter(int xch)
{
    g_ch = xch;
    ch_available = false;
    pre_rawcount = ENC_INIT_VAL;
    encount = 0;
}

int phaseCounter::getCount(){
	unsigned short int rawcount;
	int diff;
	
	if(ch_available){
	    switch(g_ch){
	        case 1:
	        rawcount =  MTU2.TCNT_1;
	        break;
	        case 2:
	        rawcount =  MTU2.TCNT_2;
	        break;
			default:
			rawcount = 0;
			break;
	    }
	    
	    diff = (int)rawcount - (int)pre_rawcount; // 差分を計算
	    if(diff > ENC_INIT_VAL){  // マイナス方向にゼロ点回ったとき
            diff = -(int)pre_rawcount - (0xFFFF - (int)rawcount);
        }else if(diff < -ENC_INIT_VAL){ // プラス方向にゼロ点回ったとき
            diff = (int)rawcount + (0xFFFF - (int)pre_rawcount);
        }
         // 差分をインクリメントする
        encount += diff;
        pre_rawcount = rawcount;
        return encount;
	}
    
    return 0;
}

void phaseCounter::setCount(int ch, int num){
	
}


void phaseCounter::init(){
	CPG.STBCR3 &= ~0x08; //マルチファンクションタイマパルスユニット2へクロックを供給(これをしていないとレジスタを書き換えられない)
	/***************ピンの設定***************/
	if(g_ch == 1){
		/***** ポートの初期化 *****/
		GPIO.PIBC1 &= ~0x0401; // ポート入力バッファ制御レジスタ 入力バッファ禁止
		GPIO.PBDC1 &= ~0x0401; // ポート双方向制御レジスタ 双方向モードを禁止
		GPIO.PM1 |= 0x0401; // ポートモードレジスタ 入力モード
		GPIO.PMC1 &= ~0x0401; // ポートモード制御レジスタ ポートモード
		GPIO.PIPC1 &= ~0x0401; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
		
		/***** 入力機能のポート設定 *****/
		GPIO.PBDC1 &= ~0x0401; // ポート双方向制御レジスタ 双方向モードを禁止
		
		/***** ポート設定 *****/
		GPIO.PFC1 |= 0x0400;
		GPIO.PFCE1 |= 0x0401;
		//GPIO.PFCAE1 &= !0xC03;

		GPIO.PIPC1 |= 0x0401; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
		GPIO.PMC1 |= 0x0401; // ポートモード制御レジスタ ポートモード
	}else if(g_ch == 2){
		/***** ポートの初期化 *****/
		GPIO.PIBC1 &= ~0x0802; // ポート入力バッファ制御レジスタ 入力バッファ禁止
		GPIO.PBDC1 &= ~0x0802; // ポート双方向制御レジスタ 双方向モードを禁止
		GPIO.PM1 |= 0x0802; // ポートモードレジスタ 入力モード
		GPIO.PMC1 &= ~0x0802; // ポートモード制御レジスタ ポートモード
		GPIO.PIPC1 &= ~0x0802; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
		
		/***** 入力機能のポート設定 *****/
		GPIO.PBDC1 &= ~0x0802; // ポート双方向制御レジスタ 双方向モードを禁止
		
		/***** ポート設定 *****/
		GPIO.PFC1 |= 0x0800;
		GPIO.PFCE1 |= 0x0802;
		//GPIO.PFCAE1 &= !0xC03;

		GPIO.PIPC1 |= 0x0802; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
		GPIO.PMC1 |= 0x0802; // ポートモード制御レジスタ ポートモード
	}

    /***************MTU1 (MTCLKA, MTCLKB)の設定***************/
    if(g_ch == 1){
	    MTU2.TSTR &= ~0x02; //MTU1.TCNTのカウント停止
	    MTU2.TCR_1 = 0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU2.TMDR_1 |= 0x04; //位相計数モード1 4逓倍のカウント読み取り
	    MTU2.TCNT_1 = ENC_INIT_VAL; //カウントを初期化
	    MTU2.TIOR_1 |= 0xAA;  //両エッジでインプットキャプチャ
	    MTU2.TSTR |= 0x02;  //MTU1.TCNTのカウント開始
	    
	    ch_available = true;
    }
    
    /***************MTU2 (MTCLKC, MTCLKD)の設定***************/
    if(g_ch == 2){
	    MTU2.TSTR &= ~0x04; //MTU1.TCNTのカウント停止
	    MTU2.TCR_2 = 0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU2.TMDR_2 |= 0x04; //位相計数モード1 4逓倍のカウント読み取り
	    MTU2.TCNT_2 = ENC_INIT_VAL; //カウントを初期化	    
	    MTU2.TIOR_2 |= 0xAA;  //両エッジでインプットキャプチャ
      	MTU2.TSTR |= 0x04;  //MTU2.TCNTのカウント開始
	    
	    ch_available = true;
    }
}

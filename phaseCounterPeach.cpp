#include "phaseCounterPeach.h"

phaseCounter::phaseCounter(int xch)
{
    g_ch = xch;
    ch_available = false;
    pre_rawcount = ENC_INIT_VAL;
    encount = 0;
    
    init_done = false;
}

int phaseCounter::getCount(){
	unsigned short int rawcount;
	int diff, n = g_ch-1;
	
	if(ch_available){
	    switch(g_ch){
	        case 1:
	        rawcount =  MTU2.TCNT_1;
	        break;
	        case 2:
	        rawcount =  MTU2.TCNT_2;
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
	
    // MSTP_MTU1=0; // これは何かわからない
    SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; // MTU0~5モジュールストップ状態解除 */
    SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; // TPU0~5モジュールストップ状態解除 */
    
    /***************ピンの設定***************/
    
    if(g_ch==1){
	    PORT2.PMR.BIT.B4=1; //P24汎用入出力機能を選択
	    PORT2.PMR.BIT.B5=1; //P25汎用入出力機能を選択
	}
	if(g_ch==2){
	    PORT2.PMR.BIT.B2=1; //P22汎用入出力機能を選択
	    PORT2.PMR.BIT.B3=1; //P23汎用入出力機能を選択
    }
    if(g_ch==3){
    	PORTC.PMR.BIT.B2=1; //PC2汎用入出力機能を選択
    	PORTC.PMR.BIT.B3=1; //PC3汎用入出力機能を選択
    }
    if(g_ch==4){
    	//PORT1.PMR.BIT.B6=1; //P16汎用入出力機能を選択
    	//PORT1.PMR.BIT.B7=1; //P17汎用入出力機能を選択
    	PORTC.PMR.BIT.B0=1; //PC0汎用入出力機能を選択
        PORTC.PMR.BIT.B1=1; //PC1汎用入出力機能を選択
    }
    
    /***************位相計数用のピンの設定***************/
    MPC.PWPR.BIT.B0WI=0; //PFSWEビットへの書き込みを許可
    MPC.PWPR.BIT.PFSWE=1; //PFSレジスタへの書き込みを許可
    
    if(g_ch==1){
	    MPC.P24PFS.BIT.PSEL=0b00010;  //P24をMTCLKAピンとして使用
	    MPC.P25PFS.BIT.PSEL=0b00010;  //P25をMTCLKBピンとして使用
    }
    if(g_ch==2){
    	MPC.P22PFS.BIT.PSEL=0b00010;  //P22をMTCLKCピンとして使用
    	MPC.P23PFS.BIT.PSEL=0b00010;  //P23をMTCLKDピンとして使用
    }
    if(g_ch==3){
    	MPC.PC2PFS.BIT.PSEL=0b00011;  //PC2をTCLKAピンとして使用
    	MPC.PC3PFS.BIT.PSEL=0b00011;  //PC3をTCLKBピンとして使用
    }
    if(g_ch==4){
    	//MPC.P16PFS.BIT.PSEL=0b00100;  //P16をTCLKCピンとして使用
    	//MPC.P17PFS.BIT.PSEL=0b00100;  //P17をTCLKDピンとして使用
    	MPC.PC0PFS.BIT.PSEL=0b00011;  //PC0をTCLKCピンとして使用
        MPC.PC1PFS.BIT.PSEL=0b00011;  //PC1をTCLKDピンとして使用
    }
    MPC.PWPR.BIT.PFSWE=0; //PFSレジスタへの書き込みを禁止
    MPC.PWPR.BIT.B0WI=1; //PFSWEビットへの書き込みを禁止
    
    /***************MTU1 (MTCLKA, MTCLKB)の設定***************/
    if(g_ch==1){
	    MTU.TSTR.BIT.CST1=0; //MTU1.TCNTのカウント停止
	    MTU1.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU1.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
	    MTU1.TCNT=ENC_INIT_VAL; //カウントを初期化
	    
	    MTU1.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
	    MTU1.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
	    MTU.TSTR.BIT.CST1=1;  //MTU1.TCNTのカウント開始
	    
	    ch_available = true;
    }
    
    /***************MTU2 (MTCLKC, MTCLKD)の設定***************/
    if(g_ch==2){
	    MTU.TSTR.BIT.CST2=0; //MTU2.TCNTのカウント停止
	    MTU2.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU2.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
	    MTU2.TCNT=ENC_INIT_VAL; //カウントを初期化
	    
	    MTU2.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
	    MTU2.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
	    MTU.TSTR.BIT.CST2=1;  //MTU2.TCNTのカウント開始
	    
	    ch_available = true;
    }    
    
	/***************TPU1 (TCLKA, TCLKB)の設定***************/
	if(g_ch==3){
		TPUA.TSTR.BIT.CST1=0; //TPU1.TCNTのカウント停止
	    TPU1.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
	    TPU1.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
	    TPU1.TCNT=ENC_INIT_VAL; //カウントを初期化
	    
	    TPU1.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
	    TPU1.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
	    TPUA.TSTR.BIT.CST1=1;  //TPU1.TCNTのカウント開始
	    
	    ch_available = true;
    }
    
    /***************TPU2 (TCLKC, TCLKD)の設定***************/
    if(g_ch==4){
    	TPUA.TSTR.BIT.CST2=0; //TPU2.TCNTのカウント停止
	    TPU2.TCR.BYTE=0;  //よくわからないけど，ここはゼロにしておけばOK?
	    TPU2.TMDR.BIT.MD=0b0100; //位相計数モード1 4逓倍のカウント読み取り
	    TPU2.TCNT=ENC_INIT_VAL; //カウントを初期化
	    
	    TPU2.TIOR.BIT.IOA=0b1010;  //両エッジでインプットキャプチャ
	    TPU2.TIOR.BIT.IOB=0b1010;  //両エッジでインプットキャプチャ
	    TPUA.TSTR.BIT.CST2=1;  //TPU2.TCNTのカウント開始
	    
	    ch_available = true;
    }
}

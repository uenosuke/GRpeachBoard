/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include "USBHostSerial.h"

#define INTERVAL 100
#define ENC_INIT_VAL 0x7FFF

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_LED_RED   , OUTPUT);
  pinMode(PIN_LED_GREEN , OUTPUT);
  pinMode(PIN_LED_BLUE  , OUTPUT);
  pinMode(PIN_LED_USER  , OUTPUT);
  pinMode(PIN_SW        , INPUT);

    Serial.println("Initializing...");
    Serial.println("Before");
    Serial.print("PIBC1: ");
    Serial.println(GPIO.PIBC1,HEX);
    Serial.print("PBDC1: ");
    Serial.println(GPIO.PBDC1,HEX);
    Serial.print("PM1: ");
    Serial.println(GPIO.PM1,HEX);
    Serial.print("PMC1: ");
    Serial.println(GPIO.PMC1,HEX);
    Serial.print("PIPC1: ");
    Serial.println(GPIO.PIPC1,HEX);

    /***************ピンの設定***************/
    /***** ポートの初期化 *****/
    GPIO.PIBC1 = 0; // ポート入力バッファ制御レジスタ 入力バッファ禁止
    GPIO.PBDC1 = 0; // ポート双方向制御レジスタ 双方向モードを禁止
    GPIO.PM1 |= 0xC03; // ポートモードレジスタ 入力モード
    GPIO.PMC1 = 0; // ポートモード制御レジスタ ポートモード
    GPIO.PIPC1 = 0; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます

    delay(100);

    Serial.println("\nAfter");
    Serial.print("PIBC1: ");
    Serial.println(GPIO.PIBC1,HEX);
    Serial.print("PBDC1: ");
    Serial.println(GPIO.PBDC1,HEX);
    Serial.print("PM1: ");
    Serial.println(GPIO.PM1,HEX);
    Serial.print("PMC1: ");
    Serial.println(GPIO.PMC1,HEX);
    Serial.print("PIPC1: ");
    Serial.println(GPIO.PIPC1,HEX);
    
    /***** 入力機能のポート設定 *****/
    Serial.println("\nSetting...");
    Serial.println("Before");
    Serial.print("PBDC1: ");
    Serial.println(GPIO.PBDC1,HEX);

    GPIO.PBDC1 &= !0xC03; // ポート双方向制御レジスタ 双方向モードを禁止
    delay(100);

    Serial.println("\nAfter");
    Serial.print("PBDC1: ");
    Serial.println(GPIO.PBDC1,HEX);

    /***** ポート設定 *****/
    Serial.println("\nPortSetting...");
    Serial.print("PFC1: ");
    Serial.println(GPIO.PFC1,HEX);
    Serial.print("PFCE1: ");
    Serial.println(GPIO.PFCE1,HEX);
    Serial.print("PFCAE1: ");
    Serial.println(GPIO.PFCAE1,HEX);
    Serial.print("PIPC1: ");
    Serial.println(GPIO.PIPC1,HEX);
    Serial.print("PMC1: ");
    Serial.println(GPIO.PMC1,HEX);

    GPIO.PFC1 |= 0xC00;
    GPIO.PFCE1 |= 0xC03;
    //GPIO.PFCAE1 &= !0xC03;

    GPIO.PIPC1 |= 0xC03; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
    GPIO.PMC1 |= 0xC03; // ポートモード制御レジスタ ポートモード
    delay(100);

    Serial.println("\nAfter");
    Serial.print("PFC1: ");
    Serial.println(GPIO.PFC1,HEX);
    Serial.print("PFCE1: ");
    Serial.println(GPIO.PFCE1,HEX);
    Serial.print("PFCAE1: ");
    Serial.println(GPIO.PFCAE1,HEX);
    Serial.print("PIPC1: ");
    Serial.println(GPIO.PIPC1,HEX);
    Serial.print("PMC1: ");
    Serial.println(GPIO.PMC1,HEX);

    CPG.STBCR3 &= !0x08; //マルチファンクションタイマパルスユニット2へのクロックを動作

    /***************MTU1 (MTCLKA, MTCLKB)の設定***************/
    //if(g_ch==1){
	    MTU2.TSTR = 0x00; //MTU1.TCNTのカウント停止
	    MTU2.TCR_1 = 0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU2.TMDR_1 |= 0x04; //位相計数モード1 4逓倍のカウント読み取り
	    MTU2.TCNT_1 = 0;//ENC_INIT_VAL; //カウントを初期化
	    
	    MTU2.TIOR_1 |= 0xAA;  //両エッジでインプットキャプチャ
	    //MTU2.TSTR |= 0x02;  //MTU1.TCNTのカウント開始
	    
	    //ch_available = true;
    //}
    
    /***************MTU2 (MTCLKC, MTCLKD)の設定***************/
    //if(g_ch==2){
	    //MTU2.TSTR &= !0x04; //MTU2.TCNTのカウント停止
	    MTU2.TCR_2 = 0;  //よくわからないけど，ここはゼロにしておけばOK?
	    MTU2.TMDR_2 |= 0x04; //位相計数モード1 4逓倍のカウント読み取り
	    MTU2.TCNT_2 = 0x000F;//ENC_INIT_VAL; //カウントを初期化	    
	    MTU2.TIOR_2 |= 0xAA;  //両エッジでインプットキャプチャ

      //CPG.STBCR3 &= !0x08; //マルチファンクションタイマパルスユニット2へのクロックの供給を停止
	    MTU2.TSTR |= 0x06;  //MTU2.TCNTのカウント開始
	    
	    //ch_available = true;
    //}      
}

void loop()
{
  unsigned short int rawcount;
	int diff;
  static unsigned short int encount = 0, pre_rawcount = 0;
	
	rawcount =  MTU2.TCNT_1;
	//rawcount =  MTU2.TCNT_2;
	    
	diff = (int)rawcount - (int)pre_rawcount; // 差分を計算
	if(diff > ENC_INIT_VAL){  // マイナス方向にゼロ点回ったとき
    diff = -(int)pre_rawcount - (0xFFFF - (int)rawcount);
  }else if(diff < -ENC_INIT_VAL){ // プラス方向にゼロ点回ったとき
    diff = (int)rawcount + (0xFFFF - (int)pre_rawcount);
  }
  // 差分をインクリメントする
  encount += diff;
  pre_rawcount = rawcount;
  
  /*while(digitalRead(PIN_SW) == 0){
    digitalWrite(PIN_LED_USER, 1);
    delay(INTERVAL);
    digitalWrite(PIN_LED_USER, 0);
    delay(INTERVAL);
  }
  digitalWrite(PIN_LED_RED, 1);
  delay(INTERVAL);
  digitalWrite(PIN_LED_RED, 0);
  digitalWrite(PIN_LED_GREEN, 1);
  delay(INTERVAL);
  digitalWrite(PIN_LED_GREEN, 0);
  digitalWrite(PIN_LED_BLUE, 1);
  delay(INTERVAL);
  digitalWrite(PIN_LED_BLUE, 0);
*/
  //if(digitalRead(A2) == HIGH) digitalWrite(PIN_LED_USER, 1);
  //else digitalWrite(PIN_LED_USER, 0);

  Serial.print("encount ");
  Serial.println(MTU2.TCNT_2);

  delay(100);

}


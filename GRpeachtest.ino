/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"
#include "phaseCounterPeach.h"
#include "AMT203VPeach.h"
#include "lpms_me1Peach.h"
#include "SDclass.h"
#include "MotionGenerator.h"

#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define INTERVAL 100
#define PIN_CSB 10

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V amt203(&SPI, PIN_CSB);
lpms_me1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;

MotionGenerator motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードで初期化

bool flag_100ms = false;

// イベントキューを作成する
//EventQueue queue;

void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void timer_warikomi(){
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  static int count_flag = 0;
  count += 2; // ここで光る周期を変えられる(はず)
  count_flag++;

  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }else{
    count = 0;
  }

  if(count_flag >= 10){
    flag_100ms = true;
    count_flag = 0;
  }
  /*Serial.print("abscount ");
  Serial.println(amt203.getEncount());*/

  /*Serial.print("\tlpms ");
  Serial.print(lpms.get_z_angle());
  Serial.print("\tencount1 ");
  Serial.print(enc1.getCount());
  Serial.print("\tencount2 ");
  Serial.println(enc2.getCount());*/
}

void error_stop(){
  while(1){
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void led_thread(){
  static int i = 0;
  if(i >= 100){
    Serial.println("Hello");
    i = 0;
  }
  i++;
}

void setup()
{
  Serial.begin(115200);
  Serial0.begin(115200);
  SERIAL_ROBOCLAW.begin(115200);
  SERIAL_LEONARDO.begin(115200);
  SERIAL_LCD.begin(115200);
  SERIAL_XBEE.begin(115200);

  SPI1.begin(); // チェック用

  pinMode(PIN_SW        , INPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_DIP1, INPUT);
  pinMode(PIN_DIP2, INPUT);
  pinMode(PIN_DIP3, INPUT);
  pinMode(PIN_DIP4, INPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  
  pinMode(PIN_SW_UP, INPUT);
  pinMode(PIN_SW_LEFT, INPUT);
  pinMode(PIN_SW_RIGHT, INPUT);
  pinMode(PIN_SW_DOWN, INPUT);
  pinMode(PIN_SW_WHITE, INPUT);
  pinMode(PIN_SW_YELLOW, INPUT);


  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  //if(amt203.init() != 1) error_stop();
  LEDblink(PIN_LED_GREEN, 2, 100);
  
  if(lpms.init() != 1) error_stop();
  LEDblink(PIN_LED_BLUE, 2, 100);
  
  // SDのカードの処理をここに入れる
  mySD.init();
  delay(10);
  Serial.println("Path reading ...");

  int actpathnum = mySD.path_read(RED, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
  Serial.print("actpathnum: ");
  Serial.println(actpathnum);
  Serial.println("Normal Start at RED zone");
  
  //Serial.println(motion.Px[0]);
  mySD.make_logfile();
  
  enc1.init();
  enc2.init();

  /***** ポートの初期化 *****/
  GPIO.PIBC4 &= ~0x0002; // ポート入力バッファ制御レジスタ 入力バッファ禁止
	GPIO.PBDC4 &= ~0x0002; // ポート双方向制御レジスタ 双方向モードを禁止
	GPIO.PM4 |= 0x0002; // ポートモードレジスタ 入力モード
	GPIO.PMC4 &= ~0x0002; // ポートモード制御レジスタ ポートモード
	GPIO.PIPC4 &= ~0x0002; // ポート IP 制御レジスタ　入出力はPMn.PMnmビットによって制御されます
  
	/***** ポート設定 *****/
	GPIO.PBDC4 &= ~0x0002; // ポート双方向制御レジスタ 双方向モードを許可
  GPIO.P4 &= ~0x0002;
	GPIO.PM4 &= ~0x0002; // ポートモード制御レジスタ ポートモード

  LEDblink(PIN_LED_RED, 2, 100);
  
  delay(500);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  // 1msごとに定期実行するイベント
  //queue.call_every(1, &led_thread);
  // イベントループが廻り続ける
  //queue.dispatch();

  //SERIAL_XBEE.println("init done");

}

void loop()
{
  /*
  // if device disconnected, try to connect it again
  if (p_cdc->connected()){
    // print characters received
    while (p_cdc->available()) {
      Serial.println(p_cdc->getc());
    }
  }
  delay(50);
  */
 while(SERIAL_LEONARDO.available()){
   Serial.print(SERIAL_LEONARDO.read());
 }

 if(digitalRead(PIN_ENC_A) == 1){
   digitalWrite(PIN_LED_1, 1);
 }else{
   digitalWrite(PIN_LED_1, 0);
 }
 if(digitalRead(PIN_ENC_B) == 1){
   digitalWrite(PIN_LED_2, 1);
 }else{
   digitalWrite(PIN_LED_2, 0);
 }
 if(digitalRead(PIN_SW_RIGHT) == 1){
   digitalWrite(PIN_LED_3, 1);
 }else{
   digitalWrite(PIN_LED_3, 0);
 }
 if(digitalRead(PIN_SW_DOWN) == 1){
   digitalWrite(PIN_LED_4, 1);
   //digitalWrite(PIN_LED_ENC, 1);
 }else{
   digitalWrite(PIN_LED_4, 0);
   //digitalWrite(PIN_LED_ENC, 0);
 }

if(flag_100ms){
  Serial.print("\tlpms ");
 Serial.print(lpms.get_z_angle());
 Serial.print("\tencount1 ");
 Serial.print(enc1.getCount());
 Serial.print("\tencount2 ");
 Serial.println(enc2.getCount());
 flag_100ms = false;
}
 

 /*digitalWrite(TESTPIN1, 1);
 digitalWrite(TESTPIN2, 1);
 digitalWrite(TESTPIN3, 1);
 digitalWrite(TESTPIN4, 1);
 digitalWrite(TESTPIN5, 1);
 digitalWrite(TESTPIN6, 1);
 digitalWrite(TESTPIN7, 1);
 digitalWrite(TESTPIN8, 1);
 digitalWrite(TESTPIN9, 1);
 delay(1000);
 digitalWrite(TESTPIN1, 0);
 digitalWrite(TESTPIN2, 0);
 digitalWrite(TESTPIN3, 0);
 digitalWrite(TESTPIN4, 0);
 digitalWrite(TESTPIN5, 0);
 digitalWrite(TESTPIN6, 0);
 digitalWrite(TESTPIN7, 0);
 digitalWrite(TESTPIN8, 0);
 digitalWrite(TESTPIN9, 0);
 delay(1000);*/
}


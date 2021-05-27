// GR-PEACHボードを使ったプログラムです
// このプログラムは，Leonardo micro pro に接続されたコントローラを使った，全方向移動ロボットの制御プログラムです．
// 駆動方式は，デュアルキャスターによる全方向移動です．
// RoboClawは本ボードに取り付けられており，RoboClawライブラリの使用を前提とします
// http://downloads.basicmicro.com/code/arduino.zip
// 作成日 2019年12月30日
// 作成者 uenosuke

#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"
#include "phaseCounterPeach.h"
#include "lpms_me1Peach.h"
#include "SDclass.h"
#include "AutoControl.h"
#include "LCDclass.h"
#include "Button.h"
#include "ManualControl.h"
#include "Platform.h"
#include "RoboClaw.h"

phaseCounter enc1(1);
phaseCounter enc2(2);

ManualControl controller;
Platform platform;

//AMT203V amt203(&SPI, PIN_CSB);
lpms_me1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);

Button button_up(PIN_SW_UP);
Button button_down(PIN_SW_DOWN);
Button button_left(PIN_SW_LEFT);
Button button_right(PIN_SW_RIGHT);
Button button_yellow(PIN_SW_YELLOW);
Button button_white(PIN_SW_WHITE);
Button dip1(PIN_DIP1);
Button dip2(PIN_DIP2);
Button dip3(PIN_DIP3);
Button dip4(PIN_DIP4);

// グローバル変数の設定
coords gPosi = {0.0, 0.0, 0.0};
//double refVx, refVy, refVz;

unsigned int ButtonState = 0, LJoyX = 127, LJoyY = 127, RJoyX = 127, RJoyY = 127; // コントローラデータ格納用

int zone; // 赤か青か
bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

// コントローラデータを取得する部分
void controller_receive(){
  static int recv_num = 0;
  static int checksum = 0;
  static char recv_msgs[9];
  char c;
  while(SERIAL_LEONARDO.available()){
    c = SERIAL_LEONARDO.read();
    if(c == '\n'){
      if(recv_num == 9){// && (checksum & 0x3F == recv_msgs[recv_num-1] - 0x20)){
        ButtonState = 0, LJoyX = 0, LJoyY = 0, RJoyX = 0, RJoyY = 0;
        ButtonState |= recv_msgs[0] - 0x20;
        ButtonState |= (recv_msgs[1] - 0x20) << 6;
        ButtonState |= (recv_msgs[2] - 0x20) << 12;
       
        LJoyX |= (recv_msgs[3] - 0x20);
        LJoyX |= ((recv_msgs[4] - 0x20) & 0x03) << 6;

        LJoyY |= ((recv_msgs[4] - 0x20) & 0x3C) >> 2;
        LJoyY |= ((recv_msgs[5] - 0x20) & 0x0F) << 4;

        RJoyX |= ((recv_msgs[5] - 0x20) & 0x30) >> 4;
        RJoyX |= ((recv_msgs[6] - 0x20) & 0x3F) << 2;

        RJoyY |= (recv_msgs[7] - 0x20);
        RJoyY |= ((recv_msgs[8] - 0x20) & 0x03) << 6;
      }
     recv_num = 0;
     //checksum = 0;
   }else{
     recv_msgs[recv_num] = c; 
     //checksum += recv_msgs[recv_num];
     recv_num++;
   }
  }
}

// setupで有効にされるタイマ割り込み処理が書いてある場所
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

  // フラグ立てるための処理
  flag_10ms = true;
  if(count_flag >= 10){
    flag_100ms = true;
    count_flag = 0;
  }

  double angle_rad;
  int encX, encY; // X,Y軸エンコーダのカウント値
  // 自己位置推定用エンコーダのカウント値取得
  encX = -enc1.getCount();
  encY =  enc2.getCount();

  // LPMS-ME1のから角度を取得
  angle_rad = (double)lpms.get_z_angle();
  gPosi = platform.getPosi(encX, encY, angle_rad);
  
}

// エラーが発生したら無限ループで停止
void error_stop(){
  myLCD.clear_display();
  myLCD.write_line("     !ERROR!", LINE_1);
  while(1){
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void setup()
{
  int lcd_line_num = LINE_0;
  bool ready_to_start = false;
  bool setting_sequence = false;
  String lcd_message = "";

  Serial.begin(115200);
  SERIAL_LEONARDO.begin(115200);
  SERIAL_LCD.begin(115200);
  SERIAL_XBEE.begin(115200);
  pinMode(PIN_XBEERESET, OUTPUT); // XBeeのリセット
  digitalWrite(PIN_XBEERESET, 0);
  delay(10);
  digitalWrite(PIN_XBEERESET,1);
  delay(10);
  
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
  analogWrite(PIN_LED_BLUE, 0);
  analogWrite(PIN_LED_GREEN, 0);
  
  myLCD.color_white(); // LCDの色を白に
  myLCD.clear_display(); // LCDをクリア

  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  //if(amt203.init() != 1) error_stop();
  LEDblink(PIN_LED_GREEN, 2, 100); // 初期が終わった証拠にブリンク

  // LPMS-ME1の初期化
  if(lpms.init() != 1) error_stop(); // 理由はわからないが，これをやる前にLEDblinkかanalogWriteを実行していないと初期化できない
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期化が終わった証拠にブリンク
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();
  
  // LCDに状態などを表示
  myLCD.write_line("MANUAL MODE", LINE_1);
  myLCD.write_line("Sensors Initialized", LINE_2);
  
  mySD.init();
  delay(10);
  //Serial.println("Path reading ...");
  myLCD.write_line("SD-card initialized", LINE_3);
  mySD.make_logfile();

  LEDblink(PIN_LED_RED, 2, 100);
  
  myLCD.write_line(">> Push A Button <<", LINE_4);
  
  // コントローラの"A"ボタンが押されるまで待機
  while(!ready_to_start){
    controller_receive();
    if(ButtonState & BUTTON_A){
      ready_to_start = true;
    }
  }

  myLCD.clear_display();
  myLCD.write_line("# Program Started  #", LINE_1);
  myLCD.write_line("pX:      pY:", LINE_2);
  
  delay(750); 

  myLCD.write_line("Angle:", LINE_3);

  myLCD.write_double(gPosi.x, LINE_2, 3);
  myLCD.write_double(gPosi.y, LINE_2, 12);
  myLCD.write_double(gPosi.z, LINE_3, 6);

  enc1.init();
  enc2.init();

  platform.platformInit(gPosi);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  controller_receive(); // コントローラ(Leonardo)からの受信

  // オンボードエンコーダやスイッチの状態に応じてLEDを制御
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

  // 10msに1回ピン情報を出力する
  if(flag_10ms){
    coords refV = controller.getRefVel(LJoyX, LJoyY, RJoyY); // ジョイスティックの値から，目標速度を生成
    platform.VelocityControl(refV); // 目標速度に応じて，プラットフォームを制御

    // SDカードにログを吐く
    String dataString = "";
    static bool first_write = true;
    if(first_write){
      dataString += "gPosix,gPosiy,gPosiz,refVx,refVy,refVz";
      mySD.write_logdata(dataString);
      first_write = false;
      dataString = "";
    }
    dataString += String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
    dataString += "," + String(refV.x, 4) + "," + String(refV.y, 4) + "," + String(refV.z, 4);

    mySD.write_logdata(dataString);
    
    // シリアル出力する
    /*SERIAL_XBEE.print("Pins: ");
    SERIAL_XBEE.print(digitalRead(PIN_DIP1));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_DIP2));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_DIP3));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_DIP4));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_UP));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_LEFT));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_RIGHT));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_DOWN));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_WHITE));
    SERIAL_XBEE.print(" ");
    SERIAL_XBEE.print(digitalRead(PIN_SW_YELLOW));

    SERIAL_XBEE.print("\tlpms ");
    SERIAL_XBEE.print(lpms.get_z_angle());
      myLCD.write_double(lpms.get_z_angle(), LINE_3, 6);
    //SERIAL_XBEE.print("abscount ");
    //SERIAL_XBEE.print(amt203.getEncount());
    SERIAL_XBEE.print("\tencount1 ");
    SERIAL_XBEE.print(enc1.getCount());
    SERIAL_XBEE.print("\tencount2 ");
    SERIAL_XBEE.println(enc2.getCount());*/

    //SERIAL_XBEE.print(refVx);
    //SERIAL_XBEE.print(" ");
    //SERIAL_XBEE.print(refVy);
    //SERIAL_XBEE.print(" ");
    //SERIAL_XBEE.println(refVz);
    //SERIAL_XBEE.print(" ");
    //SERIAL_XBEE.println(thetaDuEnc);

    Serial.print(refV.x);
    Serial.print(" ");
    Serial.print(refV.y);
    Serial.print(" ");
    Serial.print(refV.z);
    Serial.print(" ");
    Serial.println(gPosi.z);
    SERIAL_XBEE.flush();

    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    myLCD.write_double(gPosi.x, LINE_2, 3);
    myLCD.write_double(gPosi.y, LINE_2, 12);
    myLCD.write_double(gPosi.z, LINE_3, 6);
    
    flag_100ms = false;
  }
 //delayMicroseconds(100);
}


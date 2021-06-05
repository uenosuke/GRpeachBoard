// 手動操作するためのサンプルプログラムです
// Leonardo pro Micro とUSBホストシールドを使って，Elecomのゲームパッドの情報を取得します
// ジョイスティックの状態に応じてロボットの速度を制御します
// 編集者：小林(亮)

#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"

//#include "AutoControl.h"
#include "Button.h"
#include "Controller.h"
#include "LCDclass.h"
#include "lpms_me1Peach.h"
#include "ManualControl.h"
#include "phaseCounterPeach.h"
#include "Platform.h"
#include "SDclass.h"
#include "RoboClaw.h"

phaseCounter enc1(1);
phaseCounter enc2(2);

ManualControl manualCon;
Platform platform(1, 1, -1, -1); // 括弧内の引数で回転方向を変えられる

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

Controller CON;

// グローバル変数の設定
coords gPosi = {0.0, 0.0, 0.0};

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
  encX = enc1.getCount();
  encY = enc2.getCount();

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
  SERIAL_CON.begin(115200);
  SERIAL_LCD.begin(115200);
  
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
  
  // コントローラの"右"ボタンが押されるまで待機
  while(!ready_to_start){
    delay(10);
    CON.update();
    if(CON.readButton(BUTTON_RIGHT) == 2){
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

  manualCon.init();

  platform.platformInit(gPosi);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  // 10msに1回ピン情報を出力する
  if(flag_10ms){
    CON.update(); // コントローラからの受信
    
    coords refV = manualCon.getRefVel(CON.readJoyLXbyte(), CON.readJoyLYbyte(), CON.readJoyRYbyte()); // ジョイスティックの値から，目標速度を生成
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
    Serial.print(CON.getButtonState(),BIN);
    Serial.print(" ");
    Serial.print(CON.readJoyLXbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyLYbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyRYbyte());
    Serial.print(" ");
    Serial.print(refV.x);
    Serial.print(" ");
    Serial.print(refV.y);
    Serial.print(" ");
    Serial.print(refV.z);
    Serial.print(" ");
    Serial.print(gPosi.x);
    Serial.print(" ");
    Serial.print(gPosi.y);
    Serial.print(" ");
    Serial.println(gPosi.z);
    
    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    myLCD.write_double(gPosi.x, LINE_2, 3);
    myLCD.write_double(gPosi.y, LINE_2, 12);
    myLCD.write_double(gPosi.z, LINE_3, 6);
    
    flag_100ms = false;
  }
}


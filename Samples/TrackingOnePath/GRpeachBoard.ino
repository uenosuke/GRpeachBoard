// 1経路のみの軌道追従制御のサンプルプログラムです
// Elecomのゲームパッドの情報を利用します
// 作成日：2021年5月27日　作成者：ueno
// 最終修正日：2021年7月7日 編集者：ueno　※M5stackへ対応
#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"

#include "AutoControl.h"
#include "Button.h"
#include "Controller.h"
#include "LpmsMe1Peach.h"
#include "ManualControl.h"
#include "PhaseCounterPeach.h"
#include "Platform.h"
#include "SDclass.h"
#include "RoboClaw.h"

PhaseCounter enc1(1);
PhaseCounter enc2(2);
ManualControl manualCon;
AutoControl autonomous;
Platform platform(1, 1, -1, -1); // 括弧内の引数で回転方向を変えられる
Controller CON;

//AMT203V amt203(&SPI, PIN_CSB);
LpmsMe1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
bool SDwrite = false; // trueでSDカードに書き出し

coords gPosi = {0.0, 0.0, 0.0};
coords gRefV = {0.0, 0.0, 0.0};
int zone = RED;
bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;

unsigned int robotState = 0; // ロボットの状態
#define STATE_LPMS_ENABLE 0x01
#define STATE_SD_INIT     0x02
#define STATE_SD_WRITE    0x04
#define STATE_ZONE        0x08
#define STATE_READY       0x10

Button button_up(PIN_SW_UP);
Button button_down(PIN_SW_DOWN);
Button button_left(PIN_SW_LEFT);
Button button_right(PIN_SW_RIGHT);
Button button_yellow(PIN_SW_A);
Button button_white(PIN_SW_B);

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
  SERIAL_M5STACK.println("!!ERROR!");
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
  Serial.begin(115200);
  SERIAL_CON.begin(115200);
  SERIAL_M5STACK.begin(115200);
  //SERIAL_XBEE.begin(115200);
  
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
  
  pinMode(PIN_DIP1, INPUT);
  pinMode(PIN_DIP2, INPUT);
  pinMode(PIN_DIP3, INPUT);
  pinMode(PIN_DIP4, INPUT);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
  analogWrite(PIN_LED_BLUE, 0);
  analogWrite(PIN_LED_GREEN, 0);

  // LPMS-ME1の初期化
  if(lpms.init() != 1) error_stop(); // 理由はわからないが，これをやる前にLEDblinkかanalogWriteを実行していないと初期化できない
  robotState |= STATE_LPMS_ENABLE;
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期化が終わった証拠にブリンク
  SERIAL_M5STACK.println("!LPMS-ME1 init done!"); // M5stackのデバッグエリアに出力「!」を先頭に付ければ改行コードの前までを出力できる
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();
  
  if(mySD.init() == 0){
    robotState |= STATE_SD_INIT;
    SERIAL_M5STACK.println("!SD-card init done!");
  }else{
    SERIAL_M5STACK.println("!SD-card init failed!!!");
  }
  delay(10);
  
  SDwrite = digitalRead(PIN_DIP4) == 0 ? true : false;

  if(SDwrite){
    mySD.make_logfile();
    robotState |= STATE_SD_WRITE;
    SERIAL_M5STACK.println("!Log file created!");
  }

  zone = digitalRead(PIN_DIP1) == 0 ? BLUE : RED;
  int actpathnum = autonomous.init(&mySD, zone);//←mySD.path_read(BLUE, motion->Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
  Serial.print("path num: ");
  Serial.println(actpathnum + 1);
  SERIAL_M5STACK.print("!Path num: ");
  SERIAL_M5STACK.println(actpathnum + 1);
  if(zone == BLUE) robotState |= STATE_ZONE;

  autonomous.gPosiInit();
  LEDblink(PIN_LED_RED, 2, 100);
  
  //robotState |= STATE_WAIT_INPUT;
  SERIAL_M5STACK.println("!Waiting controller input");
  
  // コントローラの"右"ボタンが押されるまで待機
  while((robotState & STATE_READY) == 0){
    delay(5);
    CON.update();
    send_state();
    
    if(CON.readButton(BUTTON_RIGHT) == 2){
      robotState |= STATE_READY;
      SERIAL_M5STACK.println("!READY TO GO !!!!!!!!!!");
    }
  }

  enc1.init();
  enc2.init();

  manualCon.init();
  platform.platformInit(gPosi);
  
  autonomous.initSettings(); // これをやっていないと足回りの指令速度生成しない
  autonomous.setConvPara(0.02, 0.997); // 初期化
  autonomous.setMaxPathnum(actpathnum); // パス数の最大値

  LEDblink(PIN_LED_GREEN, 2, 100);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  // 10msに1回ピン情報を出力する
  if(flag_10ms){
    CON.update(); // コントローラからの受信
    
    //ユーザ編集部分 >>>>>>>>>>>>>>>>>

    // 軌道追従制御させるための処理 >>>>
    coords refV = autonomous.getRefVel(CON.getButtonState()); // 各目標点に対する位置決め動作を生成
    platform.VelocityControl(refV); // 目標速度に応じて，プラットフォームを制御
    // <<<<

    // シリアル出力する
    Serial.print(CON.getButtonState(),BIN);
    Serial.print(" ");
    Serial.print(CON.readJoyLXbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyLYbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyRXbyte());
    Serial.print(" ");
    Serial.print(CON.readJoyRYbyte());
    Serial.print(" ");
    Serial.print(gRefV.x);
    Serial.print(" ");
    Serial.print(gRefV.y);
    Serial.print(" ");
    Serial.print(gRefV.z);
    Serial.print(" ");
    Serial.print(gPosi.x);
    Serial.print(" ");
    Serial.print(gPosi.y);
    Serial.print(" ");
    Serial.println(gPosi.z);

    // SDカードにログを吐く
    if(SDwrite){ // 変数が追加されています!!!!
      String dataString = "";
      static bool first_write = true;
      if(first_write){
        dataString += "gPosix,gPosiy,gPosiz,gRefVx,gRefVy,gRefVz";
        mySD.write_logdata(dataString);
        first_write = false;
        dataString = "";
      }
      dataString += String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
      dataString += "," + String(gRefV.x, 4) + "," + String(gRefV.y, 4) + "," + String(gRefV.z, 4);

      mySD.write_logdata(dataString);
    }

    //ユーザ編集部分 <<<<<<<<<<<<<<<<<
    
    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    send_state(); // M5stackへ状態の送信 ※不要な場合はコメントアウトを

    //ユーザ編集部分 >>>>>>>>>>>>>>>>>

    //ユーザ編集部分 <<<<<<<<<<<<<<<<<
    
    flag_100ms = false;
  }
}

// M5stackに送るデータ
void send_state(){
  unsigned int checksum = 0;
  char sendStr[25] = {0};
  int sendaData[6] = { (int)(gPosi.x * 100),  (int)(gPosi.y * 100),  (int)(gPosi.z * 100), (int)(gRefV.x * 100),  (int)(gRefV.y * 100),  (int)(gRefV.z * 100)};
  bool flagMinus[6] = {0};

  for(int i = 0; i < 6; i++){
    if(sendaData[i] < 0){
      flagMinus[i] = true;
      sendaData[i] = abs(sendaData[i]);
    }
  }
  sendStr[0] = 0; // ゼロで状態送信を指定
  sendStr[1] = robotState;
  sendStr[2] = sendaData[0] & 0x3F;
  sendStr[3] = ( sendaData[0] >> 6 ) & 0x1F;
  if(flagMinus[0]) sendStr[3] |= 0x20;

  sendStr[4] = sendaData[1] & 0x3F;
  sendStr[5] = ( sendaData[1] >> 6 ) & 0x1F;
  if(flagMinus[1]) sendStr[6] |= 0x20;
  
  sendStr[7] = sendaData[2] & 0x3F;
  sendStr[7] = ( sendaData[2] >> 6 ) & 0x1F;
  if(flagMinus[2]) sendStr[7] |= 0x20;

  sendStr[8] = sendaData[3] & 0x3F;
  sendStr[9] = ( sendaData[3] >> 6 ) & 0x1F;
  if(flagMinus[3]) sendStr[9] |= 0x20;
  
  sendStr[10] = sendaData[4] & 0x3F;
  sendStr[11] = ( sendaData[4] >> 6 ) & 0x1F;
  if(flagMinus[4]) sendStr[11] |= 0x20;
  
  sendStr[12] = sendaData[5] & 0x3F;
  sendStr[13] = ( sendaData[5] >> 6 ) & 0x1F;
  if(flagMinus[5]) sendStr[13] |= 0x20;

  sendStr[14] = (int)(CON.readJoyLXbyte() * 0.247) & 0x3F;
  sendStr[15] = (int)(CON.readJoyLYbyte() * 0.247) & 0x3F;
  sendStr[16] = (int)(CON.readJoyRXbyte() * 0.247) & 0x3F;
  sendStr[17] = (int)(CON.readJoyRYbyte() * 0.247) & 0x3F;
  
  unsigned int ButtonState = CON.getButtonState(); // コントローラデータ格納用
  sendStr[18] = ButtonState & 0x3F;
  sendStr[19] = (ButtonState >> 6) & 0x3F;
  sendStr[20] = (ButtonState >> 12) & 0x3F; // ここはボタン数によって書き換える

  for(int i = 0; i < 21; i++){
    checksum += (unsigned int)sendStr[i];
    SERIAL_M5STACK.write(sendStr[i] + 0x20);
  }
  sendStr[20] = (char)checksum & 0x3F;
  SERIAL_M5STACK.write(sendStr[20] + 0x20);
  //SERIAL_M5STACK.write((checksum & 0x3F) + 0x20);  
  SERIAL_M5STACK.print("\n");
}
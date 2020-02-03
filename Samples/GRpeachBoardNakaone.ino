/* GR-PEACH Sketch Template V2.05.02 */
#include <Arduino.h>
#include <MsTimer2.h>

#include "define.h"
#include "phaseCounterPeach.h"
//#include "AMT203VPeach.h"
#include "lpms_me1Peach.h"
#include "SDclass.h"
#include "AutoControl.h"
#include "Platform.h"
#include "LCDclass.h"
#include "Button.h"

#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_RESET 66
#define INTERVAL 100
#define PIN_CSB 10

#define ADR_MD1             ( 128 )
#define ADR_MD2             ( 129 )

coords gPosi = {0.0, 0.0, 0.0};
coords preGposi = {0.0, 0.0, 0.0};

// 自己位置推定用のエンコーダ
phaseCounter enc1(1);
phaseCounter enc2(2);

// turntableのthetaDu用
//AMT203V amt203(&SPI, PIN_CSB);
// lpms-me1
lpms_me1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);
// RoboClaw
//RoboClaw MD(&SERIAL_ROBOCLAW,1);//10);
RoboClaw MD(&SERIAL_ROBOCLAW,1);

AutoControl Auto;
Platform platform(&MD);

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

boolean pidPreError_update = false;

int encX = 0, encY = 0;
int preEncX = 0, preEncY = 0;
int pre_tmpEncA = 0, pre_tmpEncB = 0, pre_tmpEncC = 0;
int ledcount = 0;
boolean stopRoller = false;
int swTiming = 0;

int count_10ms = 0;//0;
boolean flag_10ms = false;
boolean flag_20ms = false;
boolean flag_5s = false;
bool flag_100ms = false;

int zone; // 赤か青か
int retry_num = 0; // リトライモード 0:通常，1:リトライ1，2:リトライ2
unsigned int ButtonState = 0, LJoyX = 127, LJoyY = 127, RJoyX = 127, RJoyY = 127, buttonB = 0;

// phase で動作フェーズを管理
int phase = 0;
char cmd = 0b00000000;

double tmpPosix = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;


// グローバル変数の設定
coords refV = { 0.0, 0.0, 0.0 };
//coords gPosi = { 0.0, 0.0, 0.0 };
double angle_rad;
const double _ANGLE_DEG = 45.0;

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}

void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void controller_receive(){
  static int recv_num = 0;
  char c;
  char recv_msgs[9];
  while(SERIAL_LEONARDO.available()){
    c = SERIAL_LEONARDO.read();
    if(c == '\n'){
      if(recv_num == 9){
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

        buttonB = ButtonState & BUTTON_B;
      }
     recv_num = 0;
   }else{
     recv_msgs[recv_num++] = c; 
   }
  }
}

void timer_warikomi(){
  static int count = 0;
  static double preEncX = 0.0, preEncY = 0.0;
  static double preAngleA = 0.0, preAngleB = 0.0, preAngleC = 0.0;
  static double preVxl = 0.0, preVyl = 0.0, preVzl = 0.0;
  count++;

  // R1370
  double rawangle, diff_angle;
  static double pre_rawangle = 0.0, angle_deg = _ANGLE_DEG;//0.0;
  boolean recv_done = false;

  // RGB LED を良い感じに光らせるための処理
  static int countLED = 0;
  static int count_flag = 0;
  countLED += 2; // ここで光る周期を変えられる(はず)
  count_flag++;

  if(countLED < 255){
    analogWrite(PIN_LED_RED, countLED);
    analogWrite(PIN_LED_BLUE, 255 - countLED);
  }else if(countLED < 255 * 2){
    analogWrite(PIN_LED_GREEN, countLED - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - countLED);
  }else if(countLED < 255 * 3){
    analogWrite(PIN_LED_BLUE, countLED - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - countLED);
  }else{
    countLED = 0;
  }

  if(count_flag >= 10){
    flag_100ms = true;
    count_flag = 0;
  }

  // 自己位置推定用エンコーダのカウント値取得
  encX = -enc1.getCount();
  encY = enc2.getCount();

  angle_rad = (double)lpms.get_z_angle();

  gPosi = platform.getPosi(encX, encY, angle_rad);

  static int count_5s = 0;
  count_5s++;
  if(count_5s == 500){
    flag_5s = true;
    //phase = 1;
  }
  //if( flag_5s ){
  count_10ms++;
  if( count_10ms == 1 ){
    flag_10ms = true;
    count_10ms = 0;
  }
  //}
  static int count_20ms = 0;
  count_20ms++;
  if( count_20ms == 2 ){
    flag_20ms = true;
    count_20ms = 0;
  }
}

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
  //Serial0.begin(115200);
  SERIAL_ROBOCLAW.begin(115200);
  SERIAL_LEONARDO.begin(115200);
  SERIAL_LCD.begin(115200);
  SERIAL_XBEE.begin(115200);
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, 0);
  delay(10);
  digitalWrite(PIN_RESET,1);
  delay(10);

  //SPI1.begin(); // チェック用

  pinMode(PIN_SW, INPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  
  // DIPスイッチやタクトスイッチのpinModeはButtonクラスで記述があるため省略
  
  myLCD.color_white(); // LCDの色を白に
  myLCD.clear_display(); // LCDをクリア

  //SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  //if(amt203.init() != 1) error_stop();
  LEDblink(PIN_LED_GREEN, 2, 100); // 初期が終わった証拠にブリンク
  
  if(lpms.init() != 1) error_stop();
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期が終わった証拠にブリンク
  
  // LCDに状態などを表示
  myLCD.write_line("Sensors Initialized", LINE_1);
  lcd_message += "ZONE:";
  
  // DIPスイッチ1番のHIGH LOW によって表示を変更
  if(dip1.get_button_state()) lcd_message += "RED  ";
  else lcd_message += "BLU  ";

  lcd_message += "MODE:";
  // DIPスイッチ2番のHIGH LOW によって表示を変更(内容は適当)
  if(dip2.get_button_state()) lcd_message += "AUTO";
  else lcd_message += "MANU";

  myLCD.write_line(lcd_message, LINE_2);

  delay(750); // これ入れないと，次が表示されない

  // 選択メニューを表示
  myLCD.write_line(" Path Read", LINE_3);
  myLCD.write_line(" Settings", LINE_4); 
  lcd_line_num = myLCD.arrow_change(LINE_3);
  
  // LCD で Path read が選択されるまで繰り返す
  while(!ready_to_start){
    if(button_yellow.button_fall()){
      if(!setting_sequence){
        if(lcd_line_num == LINE_3){
          ready_to_start = true;
          if(dip1.get_button_state()){
            myLCD.write_str("RED  ", LINE_2, 5);
            zone = RED;
          }else{
            myLCD.write_str("BLU  ", LINE_2, 5);
            zone = BLUE;
          }
        }else{
          myLCD.clear_display();
          myLCD.write_line(" PID Setting", LINE_1);
          myLCD.write_line(" Velocity Setting", LINE_2);
          //SERIAL_LCD.print("                    ");
          myLCD.write_line(" Return", LINE_4);
          lcd_line_num = myLCD.arrow_change(LINE_1);
          setting_sequence = true;
        }
      }else{
        if(lcd_line_num == LINE_4){
          myLCD.clear_display();
          //myLCD.write_line(" ");
          lcd_message = "ZONE:";
          if(dip1.get_button_state()) lcd_message += "RED  ";
          else lcd_message += "BLU  ";

          lcd_message += "MODE:";
          // DIPスイッチ2番のHIGH LOW によって表示を変更(内容は適当)
          if(dip2.get_button_state()) lcd_message += "AUTO";
          else lcd_message += "MANU";

          myLCD.write_line(lcd_message, LINE_2);
          
          myLCD.write_line(" Path Read", LINE_3);
          myLCD.write_line(" Settings", LINE_4); 
          lcd_line_num = myLCD.arrow_change(LINE_3);

          setting_sequence = false;
        }
      }
    }
    else if(button_down.button_fall()){
      if(setting_sequence){
        if(lcd_line_num == LINE_1){
          lcd_line_num = myLCD.arrow_change(LINE_2);
        }else if(lcd_line_num == LINE_2){
          lcd_line_num = myLCD.arrow_change(LINE_3);
        }else if(lcd_line_num == LINE_3){
          lcd_line_num = myLCD.arrow_change(LINE_4);
        }
      }else{
        lcd_line_num = myLCD.arrow_change(LINE_4);
      }
    }
    else if(button_up.button_fall()){
      if(setting_sequence){
        if(lcd_line_num == LINE_2){
          lcd_line_num = myLCD.arrow_change(LINE_1);
        }else if(lcd_line_num == LINE_3){
          lcd_line_num = myLCD.arrow_change(LINE_2);
        }else if(lcd_line_num == LINE_4){
          lcd_line_num = myLCD.arrow_change(LINE_3);
        }
      }else{
        lcd_line_num = myLCD.arrow_change(LINE_3);
      }
    }
    
    if(dip1.button_changed()){
      if(dip1.get_button_state()) myLCD.write_str("RED  ", LINE_2, 5);
      else myLCD.write_str("BLU  ", LINE_2, 5);
    }
    if(dip2.button_changed()){
      if(dip2.get_button_state()) myLCD.write_str("AUTO", LINE_2, 15);
      else myLCD.write_str("MANU", LINE_2, 15);
    }
    delay(50); // これが無いと，タクトスイッチの立ち上がり，下がりがうまく取れない
  }
  myLCD.clear_display(); // LCDをクリア

  // SDのカードの処理をここに入れる
  mySD.init();
  delay(10);
  int button_state;
  int actpathnum;
  if(zone == RED){ // RED Field
    cmd = BIT_RED;

    actpathnum = Auto.init(&mySD, RED);//←mySD.path_read(RED, motion.Px  , motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
    Serial.print("path num: ");
    Serial.println(actpathnum);

    /*if((button_state & 0x01)){
      // 通常スタート
      Serial.println("Normal Start at RED zone");
      retry_num = 10;
      phase = 0;
    }
    else if((button_state & 0x02)){
      // リトライ1
      Serial.println("Re-start at RED zone");
      retry_num = 11;
      phase = 100;
    }*/
  }else{ // BLUE Field
    cmd = BIT_BLUE;
    
    actpathnum = Auto.init(&mySD, BLUE);//←mySD.path_read(BLUE, motion->Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
    Serial.print("path num: ");
    Serial.println(actpathnum);

    /*if(button_state & 0x01){
      // 通常スタート
      Serial.println("Normal Start at BLUE zone");
      retry_num = 0;
      phase = 0;
    }
    else if(button_state & 0x02){
      // リトライ1
      Serial.println("Re-start at BLUE zone");
      retry_num = 1;
      phase = 100;
    }*/
  }

  Auto.gPosiInit();
  platform.platformInit(gPosi);

  myLCD.write_line("### SD-card Read ###", LINE_1);
  lcd_message = "PathNum:";
  lcd_message += String(actpathnum);
  lcd_message += " ZONE:";
  if(zone == RED){
    lcd_message += "RED";
    myLCD.color_red();
  }else{
    lcd_message += "BLU";
    myLCD.color_blue();
  }
  myLCD.write_line(lcd_message, LINE_2);
  
  mySD.make_logfile();
  LEDblink(PIN_LED_RED, 2, 100);
  
  myLCD.write_line(">Push B Button<", LINE_4);//(">Push YELLOW Button<", LINE_4);
  
  // 黄色スイッチ押されるまで待機
  /*while(!button_yellow.button_rise()){
    delay(50);
  }*/
  while(!buttonB){
    controller_receive();
  }

  myLCD.clear_display();
  myLCD.write_line("# Program Started  #", LINE_1);
  myLCD.write_line("pX:      pY:", LINE_2);
  
  delay(750); 

  myLCD.write_line("Angle:", LINE_3);
  myLCD.write_line("PathN:    Phase:", LINE_4);

  myLCD.write_double(gPosi.x, LINE_2, 3);
  myLCD.write_double(gPosi.y, LINE_2, 12);
  myLCD.write_double(gPosi.z, LINE_3, 6);

  //delay(500);

  //Serial.println(motion.Px[0]);

  Auto.initSettings(); // これをやっていないと足回りの指令速度生成しない
  Auto.setConvPara(0.02, 0.997); // 初期化
  Auto.setMaxPathnum(actpathnum); // パス数の最大値
  
  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  // 自己位置推定用のエンコーダ
  enc1.init();
  enc2.init();
}


void loop()
{
  //static int dataCount = 0;
  int syusoku;
  static int wait_count = 0;
  static byte pre_buttonstate = 0;
  static int pathNum = 0  ;

  byte swState = 0b00000000;

  char state = 0b00000000;

  if( flag_10ms ){
    pathNum = Auto.getPathNum();
    preGposi = gPosi;
    //coords refV;
    int conv;

    refV = Auto.getRefVel();
    if( gPosi.x != preGposi.x || gPosi.y != preGposi.y || gPosi.z != preGposi.z ) platform.setPosi(gPosi);
    platform.VelocityControl(refV); // 目標速度に応じて，プラットフォームを制御
    
    pre_buttonstate = pre_buttonstate<<1;
    pre_buttonstate &= 0x0F;
    pre_buttonstate |= !(buttonB>>3);
    flag_10ms = false;
  }

  if( flag_20ms ){

    /*** SDカード利用のために追加　2019/05/05 ***/
    String dataString = "";
    static bool first_write = true;
    if(first_write){
      dataString += "phase,path_num,onx,ony,gPosix,gPosiy,gPosiz,angle,dist,refKakudo,refVx,refVy,refVz";
      mySD.write_logdata(dataString);
      first_write = false;
      dataString = "";
    }
    dataString += String(phase) + "," + String(Auto.getPathNum()) + "," + String(Auto.onx(), 4) + "," + String(Auto.ony(), 4);
    dataString += "," + String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
    dataString += "," + String(Auto.angle(), 4)  + "," + String(Auto.dist(), 4) + "," + String(Auto.refKakudo(), 4);
    dataString += "," + String(refV.x, 4) + "," + String(refV.y, 4) + "," + String(refV.z, 4);

    mySD.write_logdata(dataString);
    /*** SDカード利用のために追加　2019/05/05 ***/

    flag_20ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    myLCD.write_double(refV.x, LINE_2, 3);//(gPosi.x, LINE_2, 3);
    myLCD.write_double(refV.y, LINE_2, 12);//(gPosi.y, LINE_2, 12);
    myLCD.write_double(refV.z, LINE_3, 6);//(gPosi.z, LINE_3, 6);
    myLCD.write_int(pathNum, LINE_4, 6);
    myLCD.write_int(phase, LINE_4, 16);
    
    flag_100ms = false;
  }
}


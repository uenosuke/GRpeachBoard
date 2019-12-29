// このプログラムは，GR-PEACHボードの動作確認用のプログラムです
// http://downloads.basicmicro.com/code/arduino.zip
// 作成日 2019年12月28日
// 作成者 uenosuke

#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"
#include "phaseCounterPeach.h"
#include "AMT203VPeach.h"
#include "lpms_me1Peach.h"
#include "SDclass.h"
#include "MotionGenerator.h"
#include "LCDclass.h"
#include "Button.h"

#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

phaseCounter enc1(1);
phaseCounter enc2(2);

AMT203V amt203(&SPI, PIN_CSB);
lpms_me1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);

MotionGenerator motion(FOLLOW_TANGENT); // 経路追従(接線方向向く)モードで初期化

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
double gPosix = 0.0, gPosiy = 0.0, gPosiz = 0.0;
double angle_rad;
int zone = RED;
bool flag_100ms = false;

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

  if(count_flag >= 1){
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
  pinMode(PIN_LED_ENC, OUTPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  
  //Buttonクラスで記述があるため省略
  //pinMode(PIN_DIP1, INPUT);
  //pinMode(PIN_DIP2, INPUT);
  //pinMode(PIN_DIP3, INPUT);
  //pinMode(PIN_DIP4, INPUT);

  //pinMode(PIN_SW_UP, INPUT);
  //pinMode(PIN_SW_LEFT, INPUT);
  //pinMode(PIN_SW_RIGHT, INPUT);
  //pinMode(PIN_SW_DOWN, INPUT);
  //pinMode(PIN_SW_WHITE, INPUT);
  //pinMode(PIN_SW_YELLOW, INPUT);
  
  myLCD.color_white(); // LCDの色を白に
  myLCD.clear_display(); // LCDをクリア

  SPI.begin(); // ここでSPIをbeginしてあげないとちゃんと動かなかった
  // if(amt203.init() != 1) error_stop();
  LEDblink(PIN_LED_GREEN, 2, 100); // 初期が終わった証拠にブリンク
  Serial.println("AMT203V init done!");
  Serial.flush();
  
  if(lpms.init() != 1) error_stop();
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期が終わった証拠にブリンク
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();
  
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

  //delay(750); // これ入れないと，次が表示されない

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
          if(dip1.get_button_state()) myLCD.write_str("RED  ", LINE_2, 5);
          else myLCD.write_str("BLU  ", LINE_2, 5);
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
    //delay(50); // これが無いと，タクトスイッチの立ち上がり，下がりがうまく取れない
  }
  myLCD.clear_display(); // LCDをクリア

  if(dip1.get_button_state()) zone = RED;
  else zone = BLUE;

  // SDのカードの処理をここに入れる
  mySD.init();
  delay(10);
  //Serial.println("Path reading ...");
  int actpathnum = mySD.path_read(zone, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
  Serial.print("actpathnum: ");
  Serial.println(actpathnum);

  //Serial.println("Normal Start at RED zone");
  myLCD.write_line("### SD-card Read ###", LINE_1);
  lcd_message = "PathNum:";
  lcd_message += String(actpathnum);
  lcd_message += " ZONE:";
  if(zone == RED) lcd_message += "RED";
  else lcd_message += "BLU";
  myLCD.write_line(lcd_message, LINE_2);
  myLCD.color_red();

  //Serial.println(motion.Px[0]);
  mySD.make_logfile();
  
  enc1.init();
  enc2.init();

  LEDblink(PIN_LED_RED, 2, 100);
  
  myLCD.write_line(">Push YELLOW Button<", LINE_4);
  
  // 黄色スイッチ押されるまで待機
  while(!button_yellow.button_rise()){
    delay(50);
  }

  myLCD.clear_display();
  myLCD.write_line("# Program Started  #", LINE_1);
  myLCD.write_line("pX:      pY:", LINE_2);
  
  delay(750); 

  myLCD.write_line("Angle:", LINE_3);
  myLCD.write_line("PathN:    Phase:", LINE_4);

  myLCD.write_double(gPosix, LINE_2, 3);
  myLCD.write_double(gPosiy, LINE_2, 12);
  myLCD.write_double(gPosiz, LINE_3, 6);

  //delay(500);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  // 1msごとに定期実行するイベント
  //queue.call_every(1, &led_thread);
  // イベントループが廻り続ける
  //queue.dispatch();

  //SERIAL_XBEE.println("init done");

}

unsigned int ButtonState = 0, LJoyX = 127, LJoyY = 127, RJoyX = 127, RJoyY = 127;

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

       Serial.print(ButtonState, HEX);
       Serial.print("\t");
       Serial.print(LJoyX, HEX);
       Serial.print("\t");
       Serial.print(LJoyY, HEX);
       Serial.print("\t");
       Serial.print(RJoyX, HEX);
       Serial.print("\t");
       Serial.println(RJoyY, HEX);
       digitalWrite(PIN_LED_ENC, !digitalRead(PIN_LED_ENC));
     }
     recv_num = 0;
   }
   else{
     recv_msgs[recv_num++] = c; 
   }
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
    myLCD.write_double(lpms.get_z_angle(), LINE_3, 6);
  //Serial.print("abscount ");
  //Serial.print(amt203.getEncount());
  Serial.print("\tencount1 ");
  Serial.print(enc1.getCount());
  Serial.print("\tencount2 ");
  Serial.println(enc2.getCount());
  flag_100ms = false;

  Serial.print("Pins: ");
  Serial.print(digitalRead(PIN_DIP1));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_DIP2));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_DIP3));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_DIP4));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_UP));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_LEFT));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_RIGHT));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_DOWN));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_WHITE));
  Serial.print(" ");
  Serial.print(digitalRead(PIN_SW_YELLOW));

  Serial.flush();

 }
 //delayMicroseconds(100);
}


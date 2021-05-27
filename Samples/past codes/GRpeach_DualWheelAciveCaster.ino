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
#include "LCDclass.h"
#include "Button.h"
#include "RoboClaw.h"

#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_RESET 66
#define INTERVAL 100
#define PIN_CSB 10

// 自己位置推定用のエンコーダ
phaseCounter enc1(1);
phaseCounter enc2(2);

// turntableのthetaDu用
AMT203V amt203(&SPI, PIN_CSB);
// lpms-me1
lpms_me1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
myLCDclass myLCD(&SERIAL_LCD);
// RoboClaw
RoboClaw MD(&SERIAL_ROBOCLAW,1);//10);

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
// 0 : スタートゾーンで待機するフェーズ（指令値一定）
//    【フェーズ移行条件】：スイッチが押されるまで
// 1 : ベジエ曲線を，ロボットの角度が接線方向になるように追従するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 2 : ゲルゲ受け渡し後に，じわじわ動いて壁に接触するフェーズ（指令値一定）
//    【フェーズ移行条件】：すべてのリミットスイッチが押された状態になったら
// 3 : 壁から離れるフェーズ（位置制御）
//    【フェーズ移行条件】：目標位置と角度に収束したら
// 4 : 角度だけ変えるフェーズ（位置制御）
//    【フェーズ移行条件】：目標位置と角度に収束したら
// 5 : シャガイの前に移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 6 : 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
// 7 : シャガイを取ったあと，スローイングゾーンの前に移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 8 : スローイングゾーンの前でとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
// 9 : シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
// 10 : シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(11): 2個目のシャガイの前まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(12): 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(13): シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
//(14): シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(15): 3個目のシャガイの前まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(16): 所定の位置(シャガイ前)にとどまるフェーズ（位置制御）
//    【フェーズ移行条件】：シャガイハンド閉じて，持ち上げたら / スイッチを押されたら
//(17): シャガイを投げる位置まで移動するフェーズ（軌道追従制御）
//    【フェーズ移行条件】：path_num が所定の数値になったら
//(18): シャガイを投げる位置で待機するフェーズ（位置制御）
//    【フェーズ移行条件】：スイッチを押されたら
//(100): リトライ（スタート待機）
//    【フェーズ移行条件】：スイッチが押されたら phase -> 101 へ
//(101): リトライ（直進後にじわじわ動いて位置補正-->シャガイ取得・投擲）
//    【フェーズ移行条件】：一定距離進んだら phae -> 2 へ

char cmd = 0b00000000;

/* double Kakudoxl, Kakudoxr, Kakudoy, tmpKakudoy;
 double Posix, Posiy, Posiz; */
double tmpPosix = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;


// グローバル変数の設定
double gPosix, gPosiy, gPosiz;//1.5708;//0;
double refVx, refVy, refVz;
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
  encX = -enc1.getCount();// Enc1.getCount();//-Enc1.getCount();//-Enc1.getCount();	// MTU1, X//xl
  encY =  enc2.getCount();//-Enc2.getCount();//-Enc2.getCount();	// MTU2, //y
  //EncountC =  Enc3.getCount();//-Enc3.getCount();// Enc3.getCount();// Enc3.getCount();	// TPU1, Y//xr

  // 角度   encountはdoubleに型変換した方がいいかもしれない
  double angX, angY;//, lpmsRad;
  angX = (double)( encX - preEncX ) * _2PI_RES4;
  angY = (double)( encY - preEncY ) * _2PI_RES4;

  angle_rad = (double)lpms.get_z_angle();

  // tmpKakudoy += Kakudoy;

  // ローカル用(zは角度)
  double Posix, Posiy, Posiz;
  static double pre_angle_rad = angle_rad;
  double angle_diff;
  angle_diff = angle_rad - pre_angle_rad;
  Posiz = angle_diff;//( MEASURE_HANKEI_X_R * Kakudoxr - MEASURE_HANKEI_X_L * Kakudoxl ) * _0P5_MEASHD;//
  Posix = RADIUS_X * angX;//( MEASURE_HANKEI_X_L * Kakudoxl + MEASURE_HANKEI_X_R * Kakudoxr ) * 0.5;
  Posiy = RADIUS_Y * angY;// + MEASURE_HANKEI_L * Posiz;//MEASURE_HANKEI_Y * Kakudoy - MEASURE_HANKEI_L * Posiz;

  //static double tmpPosix = 0.0, tmpPosixl = 0.0, tmpPosixr = 0.0, tmpPosiy = 0.0, tmpPosiz = 0.0;
  tmpPosix += Posix;
  tmpPosiy += Posiy;
  tmpPosiz += Posiz;

  // 回転中心の座標
  /* double Posixrc, Posiyrc;
   	double _Posix, _Posiy;
   	if( fabs(Posixr - Posixl) > 0.0001){//0.001 ){	
   		Posixrc = ( Posiy  * 2.0 * MEASURE_HANKEI_D) / ( Posixr - Posixl ) + MEASURE_HANKEI_L;
   		Posiyrc = ( Posixl * 2.0 * MEASURE_HANKEI_D) / ( Posixr - Posixl ) + MEASURE_HANKEI_D;
   	
   		// 回転中心とロボットの中心までの距離(ロー)
   		double rho = sqrt(pow(Posixrc, 2.0) + pow(Posiyrc, 2.0));
   		// Posixrc, Posiyrcを回転中心として，前の座標から今の座標までの円周
   		double deltaL = rho * Posiz;
   		// グローバル用(zは角度)
   		
   		//gPosix += Posix * cos( gPosiz ) - Posiy * sin( gPosiz );//Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
   		//gPosiy += Posix * sin( gPosiz ) + Posiy * cos( gPosiz );//Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
   		//gPosiz += Posiz;
   		_Posix = deltaL * Posiyrc;
   		_Posiy = deltaL * Posixrc;
   		//gPosiz += Posiz;
   	}else{
   		_Posix = Posix;
   		_Posiy = Posiy;
   	} */

  double tmp_Posiz = gPosiz + ( Posiz * 0.5 ); // つまりgPosi + ( Posiz / 2.0 );
  gPosiz += Posiz;
  gPosix += Posix * cos( gPosiz ) - Posiy * sin( gPosiz );//Posix * cos( tmp_Posiz ) - Posiy * sin( tmp_Posiz );
  gPosiy += Posix * sin( gPosiz ) + Posiy * cos( gPosiz );//Posix * sin( tmp_Posiz ) + Posiy * cos( tmp_Posiz );
  //gPosiz += Posiz;

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

  preEncX = encX;
  preEncY = encY;

  pre_angle_rad = angle_rad;

  // pre_tmpEncA = tmpEncA;
  // pre_tmpEncB = tmpEncB;
  // pre_tmpEncC = tmpEncC;

  /*Serial.print( onx, 2 );
   	Serial.print( "\t" );
   	Serial.print( ony, 2 );
   	Serial.print( "\t" );
   	Serial.print( gPosix, 2 );
   	Serial.print( "\t" );
   	Serial.print( gPosiy, 2 );
   	Serial.print( "\t" );
   	Serial.println(phase);//( gPosiz, 2 );*/
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

  SPI1.begin(); // チェック用

  pinMode(PIN_SW, INPUT);

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
  if(zone == RED){//if( !digitalRead(27) ){	// 赤
    cmd = BIT_RED;
    //Serial1.print('L');
    //Serial1.print(cmd); // 初期化
    //Serial1.print('\n');

    actpathnum = mySD.path_read(RED, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
    Serial.println(actpathnum);
    //mySD.path_read(RED, Px_SD, Py_SD, refvel_SD, refangle_SD);

    if((button_state & 0x01)){
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
    }/*else if(button_state & 0x04 == 0x04){
     			// リトライ2
     			retry_num = 12;
     			gPosiz = 3.14159265;
     			phase = 100;
     		}*/
  }
  else{					// 青
    cmd = BIT_BLUE;
    //Serial1.print('L');
    //Serial1.print(cmd); // 初期化
    //Serial1.print('\n');

    //digitalWrite(PIN_BLUE, HIGH);
    actpathnum = mySD.path_read(BLUE, motion.Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
    Serial.print("path num: ");
    Serial.println(actpathnum);
    //mySD.path_read(BLUE, Px_SD, Py_SD, refvel_SD, refangle_SD);

    if(button_state & 0x01){
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
    }/*else if(button_state & 0x04 == 0x04){
     			// リトライ2
     			retry_num = 2;
     			gPosiz = 0.0;
     			phase = 100;
     		}*/
  }

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

  myLCD.write_double(gPosix, LINE_2, 3);
  myLCD.write_double(gPosiy, LINE_2, 12);
  myLCD.write_double(gPosiz, LINE_3, 6);

  //delay(500);

  gPosix = motion.Px[0];
  gPosiy = motion.Py[0];

  //Serial.println(motion.Px[0]);

  motion.initSettings(); // これをやっていないと足回りの指令速度生成しない
  motion.setConvPara(0.02, 0.997); // 初期化
  motion.setMaxPathnum(actpathnum); // パス数の最大値


  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();

  // 自己位置推定用のエンコーダ
  enc1.init();
  enc2.init();

  // 1msごとに定期実行するイベント
  //queue.call_every(1, &led_thread);
  // イベントループが廻り続ける
  //queue.dispatch();

  //SERIAL_XBEE.println("init done");

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

  /* if( !digitalRead
   	(PIN_SW) ){
   		reboot_function();
   	} */


  if( flag_10ms ){
    pathNum = motion.getPathNum();
    int conv;

    //------------------------------------上半身との通信------------------------------------
    //unsigned short int len = Serial1.available(); // 何byte送られてきたかわかる？
    //static char buff[10] = {
    //};  // 送られてきた数字が一文字ずつ格納
    //static int co = 0;  // buffの配列の要素数をcountしてるだけ

    //for (int i = len;i;i--){  // 送られてきた数分繰り返す

    //buff[co] = Serial1.read();
    //if(buff[co] == '\n' /*&& buff[co-2] == 'U'*/) {
    //state = buff[co-1]; // 改行コマンド送る前にstateを送ってる
    //for (int j = co;j;j--){
    //	buff[j] = 0;  // 全部終わったら0に書き換える
    //}
    //co=0;
    //}else;

    //co++;

    //if (co > 9){
    //for (int j = 0;j <= 9;j++){
    //	buff[j] = 0;
    //}
    //co=0;
    //}else;
    //}
    //------------------------------------上半身との通信------------------------------------

    ///// phase 0 /////////////////////////////////////////////////////////////////////////
    if(phase == 0){
      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;
      if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){//if(pre_buttonstate == 0x0F && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
        if(retry_num == 10){
          gPosiz = 2.35619449;
          gPosix = -0.5;
          gPosiy =  0.5;
        }
        else if(retry_num == 0){
          gPosiz = 0.785398;//1.5708;//0;
          gPosix = 0.5;
          gPosiy = 0.5;
        }
        phase = 1;
      }
      ///// phase 1 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 1){ // ゲルゲ受け渡しまで
      // ----------------------------------上半身との通信----------------------------------
      if( pathNum == 8 && cmd != BIT_DEP ){
        cmd = BIT_DEP;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      else if( pathNum == 10 && cmd != BIT_STOR ){
        cmd = BIT_STOR;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      // ----------------------------------上半身との通信----------------------------------

      if(motion.getMode() != FOLLOW_TANGENT) motion.setMode(FOLLOW_TANGENT); // 接線方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

      if(syusoku == 1){
        if( pathNum <= STATE1_1 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
          motion.setRefKakudo();

          if( pathNum == STATE1_1 ) phase = 20;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }

      ///// phase 20 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 20){
      // ----------------------------------上半身との通信----------------------------------
      if( pathNum == 8 && cmd != BIT_DEP ){
        cmd = BIT_DEP;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      else if( pathNum == 10 && cmd != BIT_STOR ){
        cmd = BIT_STOR;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      // ----------------------------------上半身との通信----------------------------------

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 接線方向を向くのではなく，指定した角度で

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

      if(syusoku == 1){
        if( pathNum <= STATE1_2 ){
          if( pathNum < STATE1_2 ){
            motion.Px[3*pathNum+3] = gPosix;
            motion.Py[3*pathNum+3] = gPosiy;
          }
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
          motion.setRefKakudo();
          
          if( pathNum == STATE1_2 ) phase = 21;//phase = 2;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 21 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 21){
      // ----------------------------------上半身との通信----------------------------------
      if( pathNum == 8 && cmd != BIT_DEP ){
        cmd = BIT_DEP;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      else if( pathNum == 10 && cmd != BIT_STOR ){
        cmd = BIT_STOR;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      // ----------------------------------上半身との通信----------------------------------

      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);
      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz);
      refVx = motion.refVx;
      refVy = motion.refVy;
      refVz = motion.refVz;
      //Serial3.print("X ");
      //Serial3.print(motion.tmpPx);
      //Serial3.print("Y ");
      //Serial3.print("Y ");
      //Serial3.println(motion.tmpPy);
      /*if(zone == BLUE){
        refVy = -0.15;
        refVx = 0.0;
      }
      else{
        refVy = 0.15;
        refVx = 0.0;
      }*/

      /*if( (!digitalRead(A0) && !digitalRead(A1)) || (!digitalRead(A4) && !digitalRead(A5)) ){
        if( pathNum <= STATE1_3 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE1_3 ) phase = 22;//phase = 2;
        }
      }*/

      // if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 接線方向を向くのではなく，指定した角度で

      // if( (!digitalRead(A0) && !digitalRead(A1)) || (!digitalRead(A4) && !digitalRead(A5)) ){
      // 	if( pathNum <= STATE1_3 ){
      // 		motion.Px[3*pathNum+3] = gPosix;
      // 		motion.Py[3*pathNum+3] = gPosiy;
      // 		motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

      // 		if( pathNum == STATE1_3 ) phase = 22;//phase = 2;
      // 	}
      // }else{
      // 	refVx = motion.refVx;
      // 	refVy = motion.refVy;
      // 	refVz = motion.refVz;
      // }
      ///// phase 22 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 22){
      // ----------------------------------上半身との通信----------------------------------
      if( pathNum == 8 && cmd != BIT_DEP ){
        cmd = BIT_DEP;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      // ----------------------------------上半身との通信----------------------------------

      if(zone == BLUE){
        refVy = -0.07;
        refVx = 0.7;
      }
      else{
        refVy = 0.1;
        refVx = 0.7;
      }

      if( fabs(gPosix) > 5.5 ){
        if( pathNum <= STATE1_4 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE1_4 ) phase = 2;
        }
      }
      ///// phase 2 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 2){ // じわじわ動いて位置補正
      if( cmd != BIT_STOR ){
        cmd = BIT_STOR;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }
      // refVx = 0.0;
      // refVy = 0.0;
      // refVz = 0.0;

      // if(pre_buttonstate == 0 && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
      // 	//gPosix = 6.075;
      // 	//gPosiy = 8.225;
      // 	//motion.Px[3*pathNum+3] = gPosix;
      // 	//motion.Py[3*pathNum+3] = gPosiy;
      // 	motion.incrPathnum(0.02, 0.997);
      // 	phase = 3;
      // }
      //byte swState = 0b00000000;
      //if( !digitalRead(A0) ) swState |= 0b00000001;	// 青用
      //if( !digitalRead(A1) ) swState |= 0b00000010;	// 青用
      //if( !digitalRead(A2) ) swState |= 0b00000100;	// 前
      //if( !digitalRead(A3) ) swState |= 0b00001000;	// 前
      //if( !digitalRead(A4) ) swState |= 0b00010000;	// 赤用
      //if( !digitalRead(A5) ) swState |= 0b00100000;	// 赤用

      refVz = 0.0;

      if(zone == BLUE){
        // サイドのスイッチが押されていた場合
        if((swState & 0b00000011) == 0b00000011) refVy = 0.0;
        else refVy = -0.15;

        // フロントのスイッチが押されていた場合
        if((swState & 0b00001100) == 0b00001100) refVx = 0.0;
        else refVx = 0.15;

        // 両方押されていた場合
        if((swState & 0b00001111) == 0b00001111){
          wait_count++;
          if(wait_count >= 50){ //0.5秒間待つ
            gPosix = 6.058;
            gPosiy = 8.245;
            gPosiz = 0.0;

            // 次の位置PIDにおける目標値
            motion.Px[3 * pathNum] = 6.475;
            motion.Py[3 * pathNum] = 8.725;

            motion.incrPathnum(0.02, 0.997);
            phase = 3;
            wait_count = 0;
          }

        } 
        else {
          wait_count = 0;
        }
      }
      else{
        // サイドのスイッチが押されていた場合
        if((swState & 0b00110000) == 0b00110000) refVy = 0.0;
        else refVy = 0.15;
        // フロントのスイッチが押されていた場合
        if((swState & 0b00001100) == 0b00001100) refVx = 0.0;
        else refVx = 0.15;

        // 両方押されていた場合
        if((swState & 0b00111100) == 0b00111100){
          wait_count++;
          if(wait_count >= 50){
            gPosix = -6.058; // ここは変えてね！
            gPosiy = 8.245;
            gPosiz = 3.1415;

            // 次の位置PIDにおける目標値　　ここも変更してね
            motion.Px[3 * pathNum] = -6.475;
            motion.Py[3 * pathNum] = 8.725;

            motion.incrPathnum(0.02, 0.997);
            phase = 3;
            wait_count = 0;
          }
        } 
        else {
          wait_count = 0;
        }
      }
    }
    ///// phase 3 /////////////////////////////////////////////////////////////////////////
    else if(phase == 3){ // 位置制御で壁から離れる
      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

      // リトライの時は速度を上げる
      if(retry_num == 1 || retry_num == 11){
        motion.refvel[pathNum] = 1.0;//1.2;
        motion.acc_count[pathNum] = 80;
      }

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        motion.Px[3*pathNum+3] = gPosix;
        motion.Py[3*pathNum+3] = gPosiy;
        //motion.refangle[pathNum+1] = 1.5708; // 次のフェーズでの目標角度
        motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

        phase = 4;
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 4 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 4){ // 位置制御で角度を変える
      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        //digitalWrite(PIN_LED2, HIGH);
        motion.Px[3*pathNum+3] = gPosix;
        motion.Py[3*pathNum+3] = gPosiy;
        motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

        phase = 5;
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 5 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 5){ // シャガイの前まで移動
      if( pathNum == 14 && cmd != BIT_DOWN ){
        cmd = BIT_DOWN;
        //Serial1.print('L');
        //Serial1.print(cmd); // シャガイを取るモード
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      if(retry_num == 1 || retry_num == 11){
        motion.refvel[pathNum] = 0.9;
        motion.acc_count[pathNum] = 15;
        motion.dec_tbe[pathNum] = 0.7;
      }

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE2 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE2 ) phase = 6;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 6 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 6){ // シャガイの前で待機(本当はシャガイハンド上げたら次のフェーズに移行)
      cmd = BIT_0;
      //Serial1.print('L');
      //Serial1.print(cmd);
      //Serial1.print('\n');

      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;

      if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
        //digitalWrite(PIN_LED2, LOW);
        if(buttonB == 1) refVx = 0.1;//if(digitalRead(A1) == 0) refVx = 0.1;//if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
      }
      else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
        //digitalWrite(PIN_LED2, HIGH);
        if(pre_buttonstate == 0x0F && buttonB == 1){//digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
          phase = 7;
        }
      }	
      ///// phase 7 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 7){ // スローイングゾーン前まで移動
      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      if(retry_num == 1 || retry_num == 11){
        motion.refvel[pathNum] = 0.9;
        motion.acc_count[pathNum] = 15;
        motion.dec_tbe[pathNum] = 0.7;
      }

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE3 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE3 ) phase = 8;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 8 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 8){ // スイッチが押されるまでスローイングゾーン前で待機

      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;

      if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){//digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
        phase = 9;
      }			
      ///// phase 9 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 9){ // 投擲位置まで移動
      if( cmd != BIT_ROT ){
        cmd = BIT_ROT;
        //Serial1.print('L');
        //Serial1.print(cmd); // 投げる
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE4 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE4 ) phase = 10;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 10 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 10){ // 投擲位置で待機(ここで投げる動作をする)
      // refVx = 0.0;
      // refVy = 0.0;
      // refVz = 0.0;
      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

      if(syusoku == 1){
        //digitalWrite(PIN_LED2, HIGH);
        //motion.Px[3*pathNum+3] = gPosix;
        //motion.Py[3*pathNum+3] = gPosiy;

        // 収束した位置にとどめるために速度をゼロにする
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;

        // wait_count++;
        //if(wait_count == 5 && cmd != BIT_EXT){
        // cmd = BIT_EXT;
        // Serial1.print('L');
        // Serial1.print(cmd); // 投げる
        // Serial1.print('\n');
        //}

        // ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？
        if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){//digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
          //digitalWrite(PIN_LED2, HIGH);
          // ボタンを押したら投擲を終了して，次のフェーズへ
          if( swTiming == 0 ){
            //digitalWrite(PIN_LED1, HIGH);
            if(cmd != BIT_EXT){
              cmd = BIT_EXT;
              //Serial1.print('L');
              //Serial1.print(cmd); // 投げる
              //Serial1.print('\n');
              swTiming++;
            }
          }
          else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
            stopRoller = true;
            if(cmd != BIT_DOWN){
              cmd = BIT_DOWN;
              //Serial1.print('L');
              //Serial1.print(cmd); // ハンド下げる
              //Serial1.print('\n');
              wait_count = 0;
              swTiming++;
            }
          }
          else{					// スイッチをもう一度押したとき
            stopRoller = false;
            phase = 11;
            motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
            swTiming = 0;
          }
        }

      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }			
      ///// phase 11 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 11){ // 2個目のシャガイの位置まで移動
      if( cmd != BIT_DOWN ){
        cmd = BIT_DOWN;
        //Serial1.print('L');
        //Serial1.print(cmd);
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE5 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE5 ) phase = 12;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 12 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 12){ // 2個目のシャガイの前で待機(ここでシャガイを取得する一連の動作を行う)
      cmd = BIT_0;
      //Serial1.print('L');
      //Serial1.print(cmd);
      //Serial1.print('\n');

      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;

      if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
        //digitalWrite(PIN_LED2, LOW);
        if(buttonB == BUTTON_B) refVx = 0.1;//if(digitalRead(A1) == 0) refVx = 0.1;//if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
      }
      else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
        //digitalWrite(PIN_LED2, HIGH);
        if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1)){//digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
          phase = 13;
        }
      }
      // if(pre_buttonstate == 0 && digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
      // 	phase = 13;
      // }			
      ///// phase 13 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 13){ // 2個目の投擲位置まで移動
      if( cmd != BIT_ROT ){
        cmd = BIT_ROT;
        //Serial1.print('L');
        //Serial1.print(cmd); // ローラ回転
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE6 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE6 ) phase = 14;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 14 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 14){ // 投擲位置で待機(ここで投げる動作をする)
      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

      if(syusoku == 1){
        //digitalWrite(PIN_LED2, HIGH);
        //motion.Px[3*pathNum+3] = gPosix;
        //motion.Py[3*pathNum+3] = gPosiy;

        // 収束した位置にとどめるために速度をゼロにする
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;

        // wait_count++;
        // if(wait_count >= 5 && cmd != BIT_EXT){
        // 	cmd = BIT_EXT;
        // 	Serial1.print('L');
        // 	Serial1.print(cmd); // 投げる
        // 	Serial1.print('\n');
        // }

        // ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？
        if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){ //digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
          // ボタンを押したら投擲を終了して，次のフェーズへ
          if( swTiming == 0 ){
            cmd = BIT_EXT;
            //Serial1.print('L');
            //Serial1.print(cmd); // 投げる
            //Serial1.print('\n');
            swTiming++;
          }
          else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
            stopRoller = true;

            cmd = BIT_DOWN;
            //Serial1.print('L');
            //Serial1.print(cmd); // ハンド下げる
            //Serial1.print('\n');
            wait_count = 0;
            swTiming++;
          }
          else{					// スイッチをもう一度押したとき
            stopRoller = false;
            phase = 15;
            motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
            swTiming = 0;
          }
        }

      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 15 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 15){ // 3個目のシャガイの位置まで移動
      if( cmd != BIT_DOWN ){
        cmd = BIT_DOWN;
        //Serial1.print('L');
        //Serial1.print(cmd);
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE7 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE7 ) phase = 16;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 16 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 16){ // 3個目のシャガイの前で待機(ここでシャガイを取得する一連の動作を行う)
      cmd = BIT_0;
      //Serial1.print('L');
      //Serial1.print(cmd);
      //Serial1.print('\n');

      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;

      if( ( MASK_SHAGAIARM & state ) == MASK_SHAGAIARM ){ // シャガイハンドが下がっている
        //digitalWrite(PIN_LED2, LOW);
        if(buttonB == BUTTON_B) refVx = 0.1;//if(digitalRead(A1) == 0) refVx = 0.1;//if(digitalRead(PIN_BUTTON1) == 0) refVx = 0.1;
      }
      else if( ( MASK_SHAGAIARM & state ) == 0b00000000 ){ // シャガイハンドが上がっている
        //digitalWrite(PIN_LED2, HIGH);
        if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1)){ //digitalRead(PIN_BUTTON1)){ // スイッチの立ち上がりを検出してフェーズ移行
          phase = 17;
        }
      }
      ///// phase 17 /////////////////////////////////////////////////////////////////////////	
    }
    else if(phase == 17){ // 投擲位置まで移動
      if( cmd != BIT_ROT ){
        cmd = BIT_ROT;
        //Serial1.print('L');
        //Serial1.print(cmd); // 投げる
        //Serial1.print('\n');
      }

      if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND); // 指定した方向を向くモードになっていなかったら変更

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
      if(syusoku == 1){
        if( pathNum <= STATE8 ){
          motion.Px[3*pathNum+3] = gPosix;
          motion.Py[3*pathNum+3] = gPosiy;
          motion.incrPathnum(0.05, 0.997);//(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値

          if( pathNum == STATE8 ) phase = 18;
        }
      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }
      ///// phase 18 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 18){ // 投擲位置で待機(ここで投げる動作をする)
      if(motion.getMode() != POSITION_PID) motion.setMode(POSITION_PID);

      syusoku = motion.calcRefvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる

      if(syusoku == 1){
        //digitalWrite(PIN_LED2, HIGH);
        //motion.Px[3*pathNum+3] = gPosix;
        //motion.Py[3*pathNum+3] = gPosiy;

        // 収束した位置にとどめるために速度をゼロにする
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;

        // wait_count++;
        // if(wait_count >= 5 && cmd != BIT_EXT){
        // 	cmd = BIT_EXT;
        // 	Serial1.print('L');
        // 	Serial1.print(cmd); // 投げる
        // 	Serial1.print('\n');
        // }

        // ボタンが押されていたら，次のフェーズへ　※場合によってはif文の外に出して，強制的に次のフェーズにした方がいいかも？

      }
      else if(syusoku == 0){ // 0の時は問題がないとき
        refVx = motion.refVx;
        refVy = motion.refVy;
        refVz = motion.refVz;
      }
      else{ // それ以外は問題ありなので止める
        refVx = 0.0;
        refVy = 0.0;
        refVz = 0.0;
      }

      if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){//digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
        // ボタンを押したら投擲を終了して，次のフェーズへ
        if( swTiming == 0 ){
          cmd = BIT_EXT;
          //Serial1.print('L');
          //Serial1.print(cmd); // 投げる
          //Serial1.print('\n');
          swTiming++;
        }
        else if( swTiming == 1 ){//if( !stopRoller ){		// スイッチを最初に押したとき
          stopRoller = true;

          cmd = BIT_DOWN;
          //Serial1.print('L');
          //Serial1.print(cmd); // ハンド下げる
          //Serial1.print('\n');
          wait_count = 0;
          swTiming++;
        }
        else{					// スイッチをもう一度押したとき
          stopRoller = false;
          //phase = 15;
          motion.incrPathnum(0.02, 0.997); // 次の曲線へ．括弧の中身は収束に使う数値
          swTiming = 0;
        }
      }

      ///// phase 100  リトライ用　 /////////////////////////////////////////////////////////////////////////			
    }
    else if(phase == 100){ // リスタート位置(線をまたいだ位置)で待機
      refVx = 0.0;
      refVy = 0.0;
      refVz = 0.0;

      if( cmd != BIT_DEP ){
        cmd = BIT_DEP;
        //Serial1.print('L');
        //Serial1.print(cmd); // ゲルゲ展開
        //Serial1.print('\n');
      }

      if(pre_buttonstate == 0x0F && buttonB == BUTTON_B){//digitalRead(A1) == 1){ //digitalRead(PIN_BUTTON1) == 1){ // スイッチの立ち上がりを検出してフェーズ移行
        if(retry_num == 11){
          gPosiz = 3.14159265;
          gPosix = -0.5;
          gPosiy =  0.5;
        }
        else if(retry_num == 1){
          gPosiz = 0.0;
          gPosix = 0.5;
          gPosiy = 0.5;
        }

        phase = 101;
        //digitalWrite(PIN_EXLED1, HIGH);
      }
      ///// phase 101  リトライ用　 /////////////////////////////////////////////////////////////////////////
    }
    else if(phase == 101){ // 投擲位置まで移動
      refVx = 0.8; // とりあえずまっすぐ進むだけ
      refVy = 0.0;
      refVz = 0.0;
      if(zone == BLUE){
        if(gPosix >= 1.7){
          cmd = BIT_STOR;
          //Serial1.print('L');
          //Serial1.print(cmd); // ゲルゲ展開
          //Serial1.print('\n');
          motion.setPathNum(11);
          phase = 2;
        }
      }
      else if(zone == RED){
        if(gPosix <= -1.7){
          cmd = BIT_STOR;
          //Serial1.print('L');
          //Serial1.print(cmd); // ゲルゲ展開
          //Serial1.print('\n');
          motion.setPathNum(11);
          phase = 2;
        }
      }
    }

    // // ベジエ曲線
    // if( mode ){//if( phase < 10 ){
    // 	static int kari = 0;
    // if( kari < 51 ){
    // 	Serial.print(kari);
    // 	Serial.print("\t");
    // 	Serial.print(Px[kari]);
    // 	Serial.print("\t");
    // 	Serial.println(Py[kari]);
    // 	kari++;
    // }
    // 	pidPreError_update = false; // 位置制御モードになったら最初だけpreErrorを現在の値で作成

    // 	if( phase < ( STATE1 - 1) ){ // スラロームからゲルゲ受け渡しまで
    // 		// 接線方向を向くモードになっていなかったら変更
    // 		if(motion.getMode() != FOLLOW_TANGENT) motion.setMode(FOLLOW_TANGENT);
    // 	}else{
    // 		// 指令した方向を向くモードになっていなかったら変更
    // 		if(motion.getMode() != FOLLOW_COMMAND) motion.setMode(FOLLOW_COMMAND);				
    // 	}

    // 	int conv = motion.calc_refvel(gPosix, gPosiy, gPosiz); // 収束していれば　1　が返ってくる
    // 	if(conv == 1){
    // 		int pathNum = motion.getPathNum();
    // 		if( pathNum < STATE1 ){
    // 			Px[3*pathNum+3] = gPosix;
    // 			Py[3*pathNum+3] = gPosiy;
    // 			motion.incrPhase(0.02, 0.997);
    // 		}else if( pathNum == STATE1 ){
    // 			flag_jiwajiwa = true;
    // 		}else{// 位置制御前は早めに次のフェーズへ
    // 			if(syusoku <= 0.3 || t_be >= 0.997){//(syusoku <= 0.25 || t_be >= 0.997){
    // 				//digitalWrite(PIN_LED2, HIGH);
    // 				Px[3*phase+3] = gPosix;
    // 				Py[3*phase+3] = gPosiy;
    // 				phase++;
    // 				pre_t_be = 0.1;


    // 				mode = false; // 位置制御モードに変更
    // 			}
    // 		}
    // 	}


    // // 位置制御　または　停止モード
    // } else {
    // 	//digitalWrite(PIN_LED1, HIGH);
    // 	// PIDのpreError更新(最初のみ)
    // 	if( !pidPreError_update ){
    // 		posiPIDx.PIDinit(Px[3*phase], gPosix);	// ref, act
    // 		posiPIDy.PIDinit(Py[3*phase], gPosiy);
    // 		posiPIDz.PIDinit(refangle[phase], gPosiz);
    // 		pidPreError_update = true; // ベジエモードに入ったらfalseになる．
    // 	}

    // 	// PIDクラスを使って位置制御を行う(速度の指令地を得る)
    // 	refVxg = posiPIDx.getCmd(Px[3*phase], gPosix, refvel[phase]);//(Px[30], gPosix, refvel[phase]);
    // 	refVyg = posiPIDy.getCmd(Py[3*phase], gPosiy, refvel[phase]);//(Py[30], gPosiy, refvel[phase]);
    // 	refVzg = posiPIDz.getCmd(refangle[phase], gPosiz, refvel[phase]);//(0.0, gPosiz, refvel[phase]);

    // 	// 上記はグローバル座標系における速度のため，ローカルに変換
    // 	refVx =  refVxg * cos(gPosiz) + refVyg * sin(gPosiz);
    // 	refVy = -refVxg * sin(gPosiz) + refVyg * cos(gPosiz);
    // 	refVz =  refVzg;

    // 	syusoku = sqrt(pow(gPosix-Px[3*phase], 2.0) + pow(gPosiy-Py[3*phase], 2.0));
    // 	if(syusoku <= 0.05){
    // 		//Px[3*phase+3] = gPosix;
    // 		//Py[3*phase+3] = gPosiy;
    // 		//phase++;
    // 		refVx = 0.0;
    // 		refVy = 0.0;
    // 		refVz = 0.0;

    // 		/* if( !digitalRead(A0) ){ // コントローラや上半身からの指令
    // 			mode = true; // ベジエモードに変更
    // 		} */
    // 	}
    // }

    // ローカル速度から，各車輪の角速度を計算
    double refOmegaR, refOmegaL, refOmegaT;
    double cosDu, sinDu, thetaDuEnc, preThetaDuEnc, thetaDu;
    thetaDuEnc = amt203.getEncount();
    if( thetaDuEnc == -1 ){
      thetaDuEnc = preThetaDuEnc;
    }
    preThetaDuEnc = thetaDuEnc;
    thetaDu = thetaDuEnc*2*PI / TT_RES4;	// 角度に変換
    cosDu = cos(thetaDu);
    sinDu = sin(thetaDu);
    refOmegaR = ( ( cosDu - sinDu ) * refVx + ( sinDu + cosDu ) * refVy ) / RADIUS_R;// right( ( cThetaDu - WPER2L_R * sThetaDu ) * refVx + ( sThetaDu + WPER2L_R * cThetaDu ) * refVy ) / RADIUS_R;
    refOmegaL = ( ( cosDu + sinDu ) * refVx + ( sinDu - cosDu ) * refVy ) / RADIUS_L;// left( ( cThetaDu + WPER2L_L * sThetaDu ) * refVx + ( sThetaDu - WPER2L_L * cThetaDu ) * refVy ) / RADIUS_L;
    refOmegaT = ( - ( 2 * sinDu / W ) * refVx + ( 2 * cosDu / W ) * refVy - refVz ) * GEARRATIO;// turntable( ( sThetaDu / L ) * refVx - ( cThetaDu / L ) * refVy + refVz ) * GEARRATIO;
    //refOmegaA = ( refVx - refVy -refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 左前
    //refOmegaB = ( refVx + refVy -refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 左後
    //refOmegaC = ( refVx - refVy +refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 右後
    //refOmegaD = ( refVx + refVy +refVz * ( _MECAHD_ADD_MECAHL ) ) / MECANUM_HANKEI;// 右前

    double mdCmdR, mdCmdL, mdCmdT;
    mdCmdR = refOmegaR * _2RES_PI;
    mdCmdL = refOmegaL * _2RES_PI;
    mdCmdT = refOmegaT * _2RES_PI_T;

    // 角度が爆速で動かないように制限(だいたい1rps)
    /*if(abs(mdCmdT) >= 20000.0){//10000.0){
      if(mdCmdT < 0.0){ // 負の値だったら
        mdCmdT = -20000.0;//10000.0;
      }
      else if(mdCmdT > 0.0){ // 正の値だったら
        mdCmdT = 20000.0;//10000.0;
      }
    }*/

    // 速度制御のためのコマンドをPIDクラスから得る
    // 最大値を超えていた場合に制限をかける

    static int dataFlag = 0;
    static int dataend = 0;

    // モータにcmd?を送り，回す
    MD.SpeedM1(ADR_MD1, -(int)mdCmdR);// 右前
    MD.SpeedM2(ADR_MD1,  (int)mdCmdL);// 左前
    MD.SpeedM1(ADR_MD2,  (int)mdCmdT);// 右後

    //Serial.print(onx * 1000);//xl
    //Serial.print("\t");
    //Serial.print(ony * 1000);//y
    //Serial.print("\t");
    /* Serial.print(gPosix * 1000);
     		Serial.print("\t");
     		Serial.println(gPosiy * 1000); */
    /* Serial.print(EncountA);
     		Serial.print("\t");
     		Serial.print(EncountB);
     		Serial.print("\t");
     		Serial.print(EncountC);
     		Serial.print("\t");
     		Serial.print(Posixl, 3);
     		Serial.print("\t");
     		Serial.println(Posixr, 3); */

    /* Serial.print(gPosix, 4);
     		Serial.print("\t");
     		Serial.print(gPosiy, 4);
     		Serial.print("\t");
     		Serial.println(gPosiz, 4); */


    /* Serial.print();
     		Serial.print();
     		Serial.print();
     		Serial.print(); */


    /* Serial.print(phase);
     		Serial.print("\t");
     		Serial.print(refVx);
     		Serial.print("\t");
     		Serial.print(refVy);
     		Serial.print("\t");
     		Serial.println(swState, BIN); */
    /* Serial.print(gPosix, 4);
     		Serial.print("\t");
     		Serial.print(gPosiy, 4);
     		Serial.print("\t");
     		Serial.println(motion.calcRefvel(gPosix, gPosiy, gPosiz)); */

    //Serial.println(digitalRead(PIN_BUTTON1));

    pre_buttonstate = pre_buttonstate<<1;
    pre_buttonstate &= 0x0F;
    pre_buttonstate |= !(buttonB>>3);//digitalRead(A1);//pre_buttonstate |= !digitalRead(PIN_BUTTON1);
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
    dataString += String(phase) + "," + String(motion.getPathNum()) + "," + String(motion.onx, 4) + "," + String(motion.ony, 4);
    dataString += "," + String(gPosix, 4) + "," + String(gPosiy, 4) + "," + String(gPosiz, 4);
    dataString += "," + String(motion.angle, 4)  + "," + String(motion.dist, 4) + "," + String(motion.refKakudo, 4);
    dataString += "," + String(refVx, 4) + "," + String(refVy, 4) + "," + String(refVz, 4);

    mySD.write_logdata(dataString);
    /*** SDカード利用のために追加　2019/05/05 ***/

    flag_20ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    myLCD.write_double(gPosix, LINE_2, 3);
    myLCD.write_double(gPosiy, LINE_2, 12);
    myLCD.write_double(gPosiz, LINE_3, 6);
    myLCD.write_int(pathNum, LINE_4, 6);
    myLCD.write_int(phase, LINE_4, 16);
    
    flag_100ms = false;
  }
}


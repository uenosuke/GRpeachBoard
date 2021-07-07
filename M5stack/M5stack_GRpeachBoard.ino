// GR-PEACH board に接続して，ロボットの状態などを表示するためのコードです

#define M5STACK_MPU6886
#include <M5Stack.h>
#include "Free_Fonts.h"

#include "define.h"
#include "DisplayControl.h"

TFT_eSprite lcd = TFT_eSprite(&M5.Lcd); // Sprite機能

coords gPosi = {0.0, 0.0, 0.0}, refVel = {0.0, 0.0, 0.0};
unsigned int robotState = 0;
unsigned int buttonState = 0;
uint8_t LJoyX = 32, LJoyY = 32, RJoyX = 32, RJoyY = 32;
int dispNum = 0;
String disp_msgs = "";

unsigned long lastCommandTick;
int requestDisplayState = 0;
bool dispNumChanged = false;

bool connectedPeach = false;

////////// タイマー関係 //////////
hw_timer_t *samplingTimer = NULL; //timer 初期化
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool t0flag = false;
bool flag100ms = false;
////////// タイマー関係 //////////

////////// 本体からの受信 //////////
void receivePeach()
{
  static int recv_num = 0;
  static char recv_msgs[25] = {};
  unsigned int checksum = 0;
  char c;

  int recvData[6] = {0};

  while (Serial2.available())
  {
    c = Serial2.read();
    if (c == '\n')
    {
      if (recv_msgs[0] == 0) {
        for (int i = 0; i < (recv_num - 1); i++) checksum += (unsigned int)recv_msgs[i] & 0x3F;
        if ((recv_num == 22) && ((checksum & 0x3F) == (unsigned int)recv_msgs[recv_num - 1]))
        {
          robotState = recv_msgs[1]; // ロボットの状態を示すデータ

          recvData[0] = (int)(recv_msgs[2] & 0x3F) | ((recv_msgs[3] & 0x1F) << 6);
          recvData[1] = (int)(recv_msgs[4] & 0x3F) | ((recv_msgs[5] & 0x1F) << 6);
          recvData[2] = (int)(recv_msgs[6] & 0x3F) | ((recv_msgs[7] & 0x1F) << 6);
          recvData[3] = (int)(recv_msgs[8] & 0x3F) | ((recv_msgs[9] & 0x1F) << 6);
          recvData[4] = (int)(recv_msgs[10] & 0x3F) | ((recv_msgs[11] & 0x1F) << 6);
          recvData[5] = (int)(recv_msgs[12] & 0x3F) | ((recv_msgs[13] & 0x1F) << 6);
          for (int i = 1; i <= 6; i++) { // 符号を付ける処理
            if (recv_msgs[i * 2] & 0x20 != 0) recvData[i] *= -1;
          }

          LJoyX = recv_msgs[14];
          LJoyY = recv_msgs[15];
          RJoyX = recv_msgs[16];
          RJoyY = recv_msgs[17];
          buttonState = recv_msgs[18] | (recv_msgs[19] << 6) | (recv_msgs[20] << 12); // コントローラのボタンのデータ
          /*Serial.print(buttonState); // デバッグ用
          Serial.print(" ");
          Serial.print(LJoyX);
          Serial.print(" ");
          Serial.println(LJoyY);*/
          gPosi.x = (double)recvData[0] / 100.0;
          gPosi.y = (double)recvData[1] / 100.0;
          gPosi.z = (double)recvData[2] / 100.0;
          refVel.x = (double)recvData[3] / 100.0;
          refVel.y = (double)recvData[4] / 100.0;
          refVel.z = (double)recvData[5] / 100.0;
        }
      }
      else if (recv_msgs[0] == 1) {
        disp_msgs = "";
        for (int i = 1; i < recv_num - 1; i++) {
          disp_msgs += (char)(recv_msgs[i] + 0x20);
        }
      }
      lastCommandTick = millis() - 1000;

      recv_num = 0;
    }
    else
    {
      recv_msgs[recv_num] = c - 0x20;
      recv_num++;
      //if (recv_num >= 21)
      //  recv_num = 0;

      lastCommandTick = millis(); // 最後に受信した時間を記録
    }
  }
}
////////// 本体からの受信 //////////

////////// タイマー割り込み処理 //////////
void IRAM_ATTR onTimer()
{
  static int count = 0;

  t0flag = true; //フラグを立てる

  count++;
  if (count == 100 / INT_TIME)
  {
    flag100ms = true;
    count = 0;
  }
}
////////// タイマー割り込み処理 //////////

// the setup routine runs once when M5Stack starts up
void setup() {

  // Initialize the M5Stack object
  M5.begin();
  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();
  M5.Speaker.begin(); // これが無いとmuteしても無意味になる
  M5.Speaker.mute(); // スピーカにノイズが乗るのでミュートする

  Serial.begin(115200);
  Serial2.begin(115200);

  // 画面出力関係 /////////////////////
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setFreeFont(FF21);
  lcd.setColorDepth(8);
  lcd.createSprite(SPRITE_W, SPRITE_H); // 逐次更新される部分をSprashで
  lcd.setFreeFont(FF21); 

  initDisplay(); // 画面の初期化
  // 画面出力関係 /////////////////////

  // 周期処理関係 /////////////////////
  samplingTimer = timerBegin(0, 80, true); //timer=1us
  timerAttachInterrupt(samplingTimer, &onTimer, true);
  timerAlarmWrite(samplingTimer, INT_TIME * 1000, true); // 10ms
  timerAlarmEnable(samplingTimer); // タイマー起動
  // 周期処理関係 /////////////////////
}

void loop() {
  static unsigned int pre_robotState = robotState; // 受信したrobotStateに変化があったときに処理をしたいのでこの変数を用意
  
  receivePeach(); // 受信処理
  
  if(robotState != pre_robotState){
    dispRobotState(); // robotStateに変化があったときはディスプレイを更新
  }
  pre_robotState = robotState;

  if (flag100ms) { // 情報の更新は100msに1回
    M5.update();
    if (M5.BtnA.wasReleased() && dispNum != 0) { // 左のボタン
      dispNum = 0; // 画面0に切り替える
      dispNumChanged = true;
    } else if (M5.BtnB.wasReleased() && dispNum != 1) { // 真ん中のボタン
      dispNum = 1; // 画面1に切り替える
      dispNumChanged = true;
    } else if (M5.BtnC.wasReleased() && dispNum != 2) { // 右のボタン
      dispNum = 2; // 画面2に切り替える
      dispNumChanged = true;
    }

    if (dispNumChanged) { // 画面番号が切り替わったときのみ処理
      initSpriteArea(); // 描画エリアの変更処理
      dispNumChanged = false;
    }

    refreshDisplay(); // 描画エリアの更新

    // 前回の受信時刻を参照して接続状態を更新
    if ((millis() - lastCommandTick) >= 1000){
      if(connectedPeach == true){
        connectedPeach = false;
        dispConnectionState(false); 
      }
    }else{
      if(connectedPeach == false){
        connectedPeach = true;
        dispConnectionState(true);
      }
    }
    flag100ms = false;
  }

  delay(1);
}
